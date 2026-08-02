// DSFE_GUI SimulationManager.cpp
#include "Scene/Object.h"
#include "Simulation/SimulationManager.h"
#include "Simulation/SimulationRenderer.h"
#include "Systems/MultiBodySystem.h"
#include "Systems/SingleBodySystem.h"

#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"

#include "Systems/RigidBodyModel.h"
#include "Systems/RigidBodySystem.h"
#include "SingleBodySystems/SingleBodySystem.h"
#include "Platform/ISimulationCore.h"

#include "DSL/IStoredProgram.h"
#include "DSL/StoredProgram.h"
#include "DSL/Parser.h"

#include <thread>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <regex>
#include <filesystem>

#include "Platform/KeyCode.h"
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace fs = std::filesystem;

namespace gui {

	// Helper: create CorePtr (unique_ptr with std::function deleter)
	static CorePtr makeCoreFactory() {
		core::ISimulationCore* raw = CreateSimulationCore_v1();
		if (!raw) { return CorePtr(nullptr, [](core::ISimulationCore*) {}); }
		// std::function deleter is constructed from the lambda implicitly
		return CorePtr(raw, [](core::ISimulationCore* p) { DestroySimulationCore(p); });
	}
	// Helper: quaternion correction for coordinate system differences (90 degrees about X-axis)
	static const glm::quat q_corr = glm::angleAxis(glm::radians(90.0f), glm::vec3(1, 0, 0));
	// Helper: convert mathlib::Vec3 to glm::vec3
	static glm::vec3 toGlm(const mathlib::Vec3& v) {
		return glm::vec3(
			static_cast<float>(v.x()),
			static_cast<float>(v.y()),
			static_cast<float>(v.z())
		);
	}
	// Helper: convert mathlib::Mat4 to glm::mat4
    static glm::mat4 toGlm(const mathlib::Mat4& m) {
        glm::mat4 g(1.0f);
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) { 
				g[c][r] = static_cast<float>(m(r, c));
			}
		}
        return g;
    }

	/*
	 * CONSTRUCTOR & DESTRUCTOR
	 */
	SimulationManager::SimulationManager() : _internalSize(1920, 1080), _displaySize(1.0f, 1.0f), _backgroundColour(0.18f, 0.18f, 0.20f),
		_backgroundAlpha(1.0f), _core(CreateSimulationCore_v1(), CoreDeleter()),
		_studyRunner(std::make_unique<StudyRunner>(makeCoreFactory, std::thread::hardware_concurrency() > 1 ? std::thread::hardware_concurrency() - 1 : 1)) {
			_sim_renderer = std::make_unique<SimulationRenderer>(_renderer);
	}
	SimulationManager::~SimulationManager() {
		if (_rendererInitialised) {
			_renderer.wait_idle();
			_renderer.shutdown();
		}
	}

	/*
	 * VULKAN RENDERER METHODS
	 */
	// Initialise the Vulkan renderer with a native window handle and dimensions
	void SimulationManager::initialiseRenderer(const renderer::NativeWindow& win, uint32_t w, uint32_t h) {
		if (_rendererInitialised) { return; }
		if (!_renderer.init(win, w, h)) {
			LOG_ERROR("Vulkan renderer initialisation failed");
			return;
		}
		_rendererInitialised = true;
	}
	// Resize the Vulkan renderer to new dimensions
	void SimulationManager::resizeRenderer(uint32_t w, uint32_t h) {
		if (!_rendererInitialised || w <= 0 || h <= 0) { return; }
		_internalSize = { static_cast<float>(w), static_cast<float>(h) };
		_renderer.resize(static_cast<uint32_t>(w), static_cast<uint32_t>(h));
	}
	// Loads a mesh from the specified file path and returns its unique ID, or INVALID_MESH_ID on failure
	uint32_t SimulationManager::load_mesh(const std::string& path) {
		if (!_rendererInitialised) { 
			LOG_ERROR("load_mesh called before renderer initialised");
			return renderer::VulkanRenderer::INVALID_MESH_ID;
		}
		assets::MeshLoader loader;
		auto meshes = loader.load(path);
		if (meshes.empty() || meshes.front()->_vertices.empty()) {
			LOG_ERROR("no geometry found in mesh file: %s", path.c_str());
			return MeshStore::INVALID_ID;
		}

		scene::Mesh mesh = *meshes.front();
		std::vector<uint32_t> indices(mesh._indices.begin(), mesh._indices.end());
		const uint32_t cpu_id = _mesh_store.add(std::move(mesh));
		const uint32_t gpu_id = _sim_renderer->upload(_mesh_store.get(cpu_id)->_vertices, indices);
		
		if (cpu_id != gpu_id) {
			LOG_ERROR("CPU mesh ID (%u) does not match GPU mesh ID (%u) for mesh: %s", cpu_id, gpu_id, path.c_str());
		}
		
		_loaded_mesh_ids.push_back(cpu_id);
        _scene.add_renderable(cpu_id, glm::mat4(1.0f));   // at origin for now
        LOG_INFO("Mesh loaded [id %u]: %s", cpu_id, path.c_str());
        return cpu_id;

	}

	/*
	 * RIGID BODY SYSTEM MANAGEMENT
	 */
	// Returns true if a rigid body is currently loaded in the simulation core, false otherwise
    const bool SimulationManager::hasRigidBody() const { return _core && _core->hasRigidBody(); }
	// Returns a reference to the currently loaded rigid body system in the simulation core.
    systems::RigidBodySystem& SimulationManager::rigidBodySystem() { return _core->rigidBodySystem(); }
	// Returns true if a rigidbody joint is to be followed in the Follow view, false otherwise. (NOT USED)
    bool SimulationManager::followRigidBodyJoint(const std::string&, const glm::vec3&) { return false; }
	// Loads a rigid body from the specified file path into the simulation core and updates the scene with its representation
	void SimulationManager::load_rigidBody(const std::string& name) {
		if (!_core) {
			LOG_ERROR("Simulation core not initialised, cannot load rigidBody");
			return;
		}
		if (!_rendererInitialised) {
			LOG_ERROR("Renderer not initialised, cannot load rigidBody");
			return;
		}
		_core->loadRigidBody(name);
		if (!_core->hasRigidBody()) {
			LOG_ERROR("Failed to load rigidBody: %s", name.c_str());
			return;
		}

		const auto& model = _core->rigidBodySystem().model();
		auto world_src = [this]() -> const std::vector<mathlib::Mat4>& { 
			return _core->rigidBodySystem().worldTransforms();
		};
		_systems.add(std::make_unique<MultiBodySystem>(model, world_src, _mesh_store, *_sim_renderer), _scene);
		_core->clearRigidBodyPresentationDirty();
		_currentRigidBodyName = model.name;
		_currentRigidBodyPath = name;
		LOG_INFO("RigidBody loaded: %s", model.name.c_str());
	}
	// Resets the rigidBody system to its initial position
	void SimulationManager::resetRigidBody() {
		if (!_core) { LOG_ERROR("Simulation core not initialised, cannot reset rigidBody"); return; }
		_core->resetRigidBody();
	}
	// Clears the rigidBody system and removes all associated objects from the scene
	void SimulationManager::clearRigidBody() { _systems.clear_all(_scene); _scene.clear(); }
	// Returns true if the currently loaded rigid body system contains a free body joint, false otherwise
	bool SimulationManager::isFreeBody() const {
		if (!_core || !_core->hasRigidBody()) { return false; }
		const auto& model = _core->rigidBodySystem().model();
		for (const auto& j : model.joints) { if (j.type == systems::eJointType::FREE) { return true; } }
		return false;
	}

	/*
	 * MULTIPLE RIGID BODY MANAGEMENT
	 */
	// Returns the number of loaded rigid bodies in the simulation core, or 0 if no core is present
	std::size_t SimulationManager::bodyCount() const {
		if (!_core) { return 0; }
		return _core->bodyCount();
	}
	// Returns a reference to the rigid body system at the specified index, allowing for manipulation of its state and properties
	systems::RigidBodySystem& SimulationManager::body(int i) {
		if (!_core) { throw std::runtime_error("Simulation core not initialised, cannot access body"); }
		return _core->body(i);
	}
	// Returns a const reference to the rigid body system at the specified index, allowing read-only access to its state and properties
	const systems::RigidBodySystem& SimulationManager::body(int i) const {
		if (!_core) { throw std::runtime_error("Simulation core not initialised, cannot access body"); }
		return _core->body(i);
	}
	// Returns the index of the currently active rigid body in the simulation core, or -1 if no core is present
	int SimulationManager::activeBodyIdx() const {
		if (!_core) { return -1; }
		return _core->activeBodyIdx();
	}
	// Sets the active rigid body index in the simulation core, allowing for switching between multiple loaded rigid bodies
	void SimulationManager::setActiveBody(int i) {
		if (!_core) { LOG_ERROR("Simulation core not initialised, cannot set active body"); return; }
		_core->setActiveBody(i);
	}
	// Clears all loaded rigid bodies from the simulation core and resets the active body index
	void SimulationManager::clearBodies() {
		if (!_core) { LOG_ERROR("Simulation core not initialised, cannot clear bodies"); return; }
		_core->clearBodies();
	}

	/*
	 * SIMULATION TICK & RENDER
	 */
	// Method to tick the simulation core, advancing the simulation state by the specified time step. This is typically called once per frame or at a fixed interval.
	void SimulationManager::tick(double dt) {
		_core->tick(dt);
	}
	// Method to render the current simulation state to the viewport. This method should be called after tick() to visualize the updated simulation state.
	void SimulationManager::setDisplaySize(uint32_t w, uint32_t h) {
		if (w <= 0.0f || h <= 0.0f) return;
		_displaySize = { w, h };
	}
	// Method to render the current simulation state to the viewport. This method should be called after tick() to visualize the updated simulation state.
	void SimulationManager::renderViewport(uint32_t w, uint32_t h) {
		if (!_rendererInitialised || w <= 0 || h <= 0) { return; }
		if (hasCompletedStudy()) {
			for (const auto& r : consumeCompletedStudy()) {
				LOG_INFO("Study completed: %s", r.tag.c_str());
			}
		}
		_systems.update_all(_scene);
		_camera.setAspect(static_cast<float>(w) / static_cast<float>(h));
		_renderer.render(_scene, _camera.getViewMatrix(), _camera.getProjection());
	}
	// Method to apply a render profile to the simulation renderer, updating its settings and resolution preset. (TO BE REMOVED)
    void SimulationManager::applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r) {
        _settingsCurrent = s;
        _resCurrent = r;
        _settingsValid = true;
        // TODO: push to VulkanRenderer once it has a settings path
    }
	void SimulationManager::setSelectedObject(scene::Object* obj) { _selectedObject = obj; }
    scene::Object* SimulationManager::getObject() { return _selectedObject; }
    std::vector<std::unique_ptr<scene::Object>>& SimulationManager::getObjects() { return _objects; }

    void SimulationManager::removeObject(scene::Object* obj) {
        if (!obj) { return; }
        if (_selectedObject == obj) { _selectedObject = nullptr; }
        std::erase_if(_objects, [obj](const std::unique_ptr<scene::Object>& p) { return p.get() == obj; });
    }

    /*
	 * SIMULATION STUDY MANAGEMENT
	 */
	void SimulationManager::pushCompletedStudies(std::vector<StudyResult>) { _hasCompletedStudy = true; }
    void SimulationManager::pushCompletedStudy(StudyResult)                { _hasCompletedStudy = true; }
    bool SimulationManager::hasCompletedStudy() const                      { return _hasCompletedStudy; }
	// Consume and return the completed study results, clearing the internal flag.
    std::vector<StudyResult> SimulationManager::consumeCompletedStudy() {
        std::vector<StudyResult> copy;
        _hasCompletedStudy = false;
        return copy;
    }
	// Start the simulation.
	void SimulationManager::startSimulation() {
        if (!hasRigidBody()) { LOG_WARN("Cannot start simulation: no rigidBody loaded"); return; }
        _core->startSimulation();
    }
	// Stop the simulation
	void SimulationManager::stopSimulation() { _core->stopSimulation(); }
	// Check if the simulation is currently running
	bool SimulationManager::isSimRunning() const { return _core->isSimRunning(); }
	// Setter for current simulation time (in seconds)
	void SimulationManager::setSimTime(double time) { _core->setSimTime(time); }
	double SimulationManager::simTime() const { return _core->simTime(); }
	// Setter and gettter for fixed timstep (in seconds)
	void SimulationManager::setFixedDt(double dt) { _core->setFixedDt(dt); }
	double SimulationManager::fixedDt() const { return _core->fixedDt(); }
	// Setter and getter for telemetry frequency (in Hz)
	void SimulationManager::setTelemetryHz(double hz) { _core->setTelemetryHz(hz); }
	double SimulationManager::telemetryHz() const { return _core->telemetryHz(); }
	// Set whether a script is currently running (used to disable UI elements, etc.)
	void SimulationManager::setScriptRunning(bool running) { _core->setScriptRunning(running); }
	bool SimulationManager::isScriptRunning() const { return _core->isScriptRunning(); }
	// Setters and getters for last script text
	void SimulationManager::setLastScriptText(const std::string& text) { _core->setLastScriptText(text); }
	std::string& SimulationManager::lastScriptText() const { return _core->lastScriptText(); }
	// Accessors for the Simulation Core's telemetry data
	diagnostics::TelemetryRecorder& SimulationManager::telemetry() { return _core->telemetry(); }
	const diagnostics::TelemetryRecorder& SimulationManager::telemetry() const { return _core->telemetry(); }
	// Accesors for the active program (if any)
	void SimulationManager::setActiveProgram(dsl::IStoredProgram* program) { _core->setActiveProgram(program); }
	dsl::IStoredProgram* SimulationManager::activeProgram() { return _core->activeProgram(); }
	const dsl::IStoredProgram* SimulationManager::activeProgram() const { return _core->activeProgram(); }
	// Access the simulation core interface (non-const and const versions)
	core::ISimulationCore* SimulationManager::simCore() { return _core.get(); }
	const core::ISimulationCore* SimulationManager::simCore() const { return _core.get(); }
	// Set the integrator method for the current simulation run
	void SimulationManager::setIntegrationMethod(integration::eIntegrationMethod method) {
		LOG_INFO("DEBUG -> Setting integration method to: %d", static_cast<int>(method));
		_core->setIntegrationMethod(method);
	}
	const integration::eIntegrationMethod SimulationManager::integrationMethod() const {
		LOG_INFO("DEBUG -> Current integration method: %d", static_cast<int>(_core->integrationMethod()));
		return _core->integrationMethod();
	}
	void SimulationManager::setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) {
		LOG_INFO("DEBUG -> Setting AutoDiff integration method to: %d", static_cast<int>(method));
		_core->setADIntegrationMethod(method);
	}
	const integration::eAutoDiffIntegrationMethod SimulationManager::autoDiffIntegrationMethod() const {
		LOG_INFO("DEBUG -> Current AutoDiff integration method: %d", static_cast<int>(_core->autoDiffIntegrationMethod()));
		return _core->autoDiffIntegrationMethod();
	}
	// Accessors for the integration method name and AutoDiff settings
	std::string SimulationManager::integrationMethodName() const { return _core->integrationMethodName(); }
	void SimulationManager::enableAutoDiff(bool enable) { _core->enableAutoDiff(enable); }
	bool SimulationManager::autoDiffEnabled() const { return _core->autoDiffEnabled(); }
	// Accessors for Physics and Dynamics state
	void SimulationManager::setGravity(const glm::vec3& g) { _core->setGravity(mathlib::Vec3(g.x, g.y, g.z)); }
	glm::vec3 SimulationManager::gravity() const { mathlib::Vec3 g = _core->gravity(); return toGlm(g); }
	// This seems to be the better solution?
	static std::string replaceIntegratorInScript(const std::string& script, const std::string& methodName) {
		std::regex re(R"((?i)set\s*\(\s*integrator\s*,\s*([a-z0-9_]+)\s*\))"); // case-insensitive regex to match my DSL command -> set(integrator, method)
		std::string replacement = "set(integrator, " + methodName + ")";
		return std::regex_replace(script, re, replacement);
	}
	// Run a script to completion synchronously with a specific integrator
	bool SimulationManager::runScriptToCompletion(const std::string& scriptText, integration::eIntegrationMethod method) {
		if (!hasRigidBody()) { return false; }

		// Map method enum to string name
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45", "implicit_euler", "implicit_midpoint", "glrk2", "glrk3" };
		const std::string methodName = names[static_cast<int>(method)];

		// Replace the integrator method in the script text
		std::string modifiedScript = replaceIntegratorInScript(scriptText, methodName);

		// Create program and parser (bound to headless core)
		auto program = std::make_unique<dsl::StoredProgram>(_core.get());
		//if (scene::Object* o = getObject()) program->setDefaultObject(o);
		auto parser = std::make_unique<dsl::Parser>(program.get());

		// Parse the modified script and start the program
		parser->parse(modifiedScript);
		program->start();
		return _core->runScriptToCompletion(program.get(), method); // this will block until the script finishes
	}

	/*
	 * INPUT HANDLING
	 */
	// Handle movement key input (WASD, Space, Shift) for camera control.
    void SimulationManager::processMovementKey(int, float) {}
	// Handle mouse movement input for camera control.
    void SimulationManager::handleContinuousMovement(const std::unordered_set<gui::eKeyCode>& keys, float dt) {
        const float speed = 3.0f * dt;
        if (keys.count(gui::eKeyCode::W))      { _camera.moveForward(speed); }
        if (keys.count(gui::eKeyCode::S))      { _camera.moveBackward(speed); }
        if (keys.count(gui::eKeyCode::D))      { _camera.moveRight(speed); }
        if (keys.count(gui::eKeyCode::A))      { _camera.moveLeft(speed); }
        if (keys.count(gui::eKeyCode::Space))  { _camera.moveUp(speed); }
        if (keys.count(gui::eKeyCode::LShift)) { _camera.moveDown(speed); }
    }
	// Handle mouse look input for camera control, updating the camera's orientation based on mouse movement deltas.
    void SimulationManager::handleMouseLook(double dx, double dy, bool captured) {
        if (!captured) { return; }
        _camera.processMouseMovement(static_cast<float>(dx), static_cast<float>(dy));
    }
	// Handle mouse wheel input for camera zoom control, adjusting the camera's field of view based on the scroll delta.
    void SimulationManager::onMouseWheel(double delta) {
        _camera.onMouseWheel(delta);
    }
	// Reset the mouse delta state, typically called when the mouse is first captured or released to prevent sudden jumps in camera orientation.
    void SimulationManager::resetMouseDelta() { _firstMouse = true; }

	/*
	 * WORKSPACE MANAGEMENT
	 */
	// Close the current workspace, clearing all systems, scene objects, and meshes. This is typically called before loading a new workspace or rigidBody.
    void SimulationManager::closeWorkspace() {
        // Order matters: 
		// 1. Systems first (they hold renderable indices)
		// 2. Scene
		// 3. Both mesh registries together (so ids realign from zero)
        _systems.clear_all(_scene);
        _scene.clear();
        _renderer.destroy_all_meshes();
        _mesh_store.clear();
        _currentRigidBodyName.clear();
		_currentRigidBodyPath.clear();
        _core->setScriptRunning(false);
        _core->stopSimulation();
        _core->setSimTime(0.0);
		_core->trajectoryManager().clearAll();
        LOG_INFO("Workspace closed");
    }
	// Apply a workspace, populating the simulation manager with the saved state. This is typically called after closeWorkspace() to load a new workspace.
    void SimulationManager::applyWorkspace(const gui::WorkspaceData& w) {
        enableAutoDiff(w.autoDiff);
        setIntegrationMethod(static_cast<integration::eIntegrationMethod>(w.integrationMethod));
        setADIntegrationMethod(static_cast<integration::eAutoDiffIntegrationMethod>(w.adIntegrationMethod));
        setFixedDt(w.simDt);
        setTelemetryHz(1.0 / w.telemetryDt);
		setGravity(w.gravity);

        _camera.setPosition(w.cameraPos);
        _camera.setYaw(w.cameraYaw);
        _camera.setPitch(w.cameraPitch);
        if (!w.rigidBodyPath.isEmpty()) {
            load_rigidBody(w.rigidBodyPath.toStdString());   // Core re-load or skip; GUI visuals rebuilt fresh
        }
        LOG_INFO("Workspace applied: '%s'", w.name.toUtf8().constData());
    }
	// Gather the current workspace state, filling the provided WorkspaceData structure with the current camera position, orientation, and rigidBody name
    void SimulationManager::gatherWorkspace(gui::WorkspaceData& w) const {
        w.rigidBodyName = QString::fromStdString(_currentRigidBodyName);
		w.rigidBodyPath = QString::fromStdString(_currentRigidBodyPath);
        w.integrationMethod = static_cast<int>(integrationMethod());
        w.adIntegrationMethod = static_cast<int>(autoDiffIntegrationMethod());
        w.autoDiff = autoDiffEnabled();
        w.simDt = fixedDt();
        w.telemetryDt = 1.0 / telemetryHz();
		w.gravity = gravity();
        w.cameraPos = _camera.getPosition();
        w.cameraYaw = _camera.getYaw();
        w.cameraPitch = _camera.getPitch();
    }
	// Set whether the simulation is currently in a manipulating state.
	void SimulationManager::setManipulating(bool on) { _core->setManipulating(on); }
    bool SimulationManager::isManipulating() const { return _core->isManipulating(); }
	// Set an external force on a specific link in the rigidBody system.
    bool SimulationManager::setLinkExternalForce(const std::string& link, const glm::vec3& p, const glm::vec3& f) {
        return _core->setLinkExternalForce(link, mathlib::Vec3(p.x, p.y, p.z), mathlib::Vec3(f.x, f.y, f.z));
    }
	// Clear all external forces applied to the rigidBody system.
	void SimulationManager::clearExternalForces() { _core->clearExternalForces(); }
	// Get the world transforms of all links in the rigidBody system, returning a const reference to a vector of 4x4 matrices representing the transforms.
    const std::vector<mathlib::Mat4>& SimulationManager::linkWorldTransforms() const { return _core->linkWorldTransforms();  }
	// Get the names of all links in the rigidBody system, returning a vector of strings representing the link names.
    std::vector<std::string> SimulationManager::linkNames() const { return _core->linkNames(); }
	// Set a highlight color for a specific link in the rigidBody system. This is typically used to visually indicate selection or focus on a particular link in the GUI.
	void SimulationManager::setLinkHighlight(const std::string& link, bool on) {
		const auto names = linkNames();
		int idx = -1;
		for (size_t i = 0; i < names.size(); ++i) { if (names[i] == link) { idx = (int)i; break; } }
		if (idx <= 0) { return; }
		if (on) {
			if (const auto* r = _scene.renderable((uint32_t)idx)) {
				_highlightIdx = idx;
				_highlightAlbedo0 = glm::vec3(r->albedo);
				_highlightMat0 = r->material;
			}
			_scene.set_material(
				(uint32_t)idx, glm::vec3(0.6f, 1.0f, 0.3f),
			    _highlightMat0.x, _highlightMat0.y, _highlightMat0.z
			);
		}
		else if (_highlightIdx == idx) {
			// Restore.
			_scene.set_material(
				(uint32_t)idx,
				_highlightAlbedo0, _highlightMat0.x, _highlightMat0.y, _highlightMat0.z
			);
			_highlightIdx = -1;
		}
	}
}
