// DSFE_GUI SimulationManager.cpp
#include "Scene/Object.h"
#include "Simulation/SimulationManager.h"
#include "Simulation/SimulationRenderer.h"
#include "Systems/MultiBodySystem.h"
#include "Systems/SingleBodySystem.h"

#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"

#include "Robots/RobotModel.h"
#include "Robots/RobotSystem.h"
#include "SingleBodySystems/SingleBodySystem.h"
#include "Platform/ISimulationCore.h"

#include "Interpreter/IStoredProgram.h"
#include "Interpreter/StoredProgram.h"
#include "Interpreter/Parser.h"

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

	static const glm::quat q_corr = glm::angleAxis(glm::radians(90.0f), glm::vec3(1, 0, 0));

    static glm::mat4 toGlm(const mathlib::Mat4& m) {
        glm::mat4 g(1.0f);
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) { 
				g[c][r] = static_cast<float>(m(r, c));
			}
		}
        return g;
    }

	// --------------------------------------------------
	//				CONSTRUCTOR & DESTRUCTOR
	// --------------------------------------------------

	SimulationManager::SimulationManager() : _internalSize(1920, 1080), _displaySize(1.0f, 1.0f), _backgroundColour(0.18f, 0.18f, 0.20f),
		_backgroundAlpha(1.0f), _core(CreateSimulationCore_v1(), CoreDeleter()),
		_studyRunner(std::make_unique<StudyRunner>(makeCoreFactory, std::thread::hardware_concurrency() > 1 ? std::thread::hardware_concurrency() - 1 : 1)) {
			_sim_renderer = std::make_unique<SimulationRenderer>(_renderer);
	}

	// Cleans up OpenGL resources
	SimulationManager::~SimulationManager() {
		if (_rendererInitialised) {
			_renderer.wait_idle();
			_renderer.shutdown();
		}
	}

	/*
	--------------------------------------------------
				 VULKAN RENDERER METHODS
	--------------------------------------------------
	*/

	void SimulationManager::initialiseRenderer(void* nativeWindowHandle) {
		if (_rendererInitialised) { return; }
		if (!_renderer.init(nativeWindowHandle)) {
			LOG_ERROR("Vulkan renderer initialisation failed");
			return;
		}
		_rendererInitialised = true;
	}

	void SimulationManager::resizeRenderer(int w, int h) {
		if (!_rendererInitialised || w <= 0 || h <= 0) { return; }
		_internalSize = { static_cast<float>(w), static_cast<float>(h) };
		_renderer.resize(static_cast<uint32_t>(w), static_cast<uint32_t>(h));
	}

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

	void SimulationManager::load_robot(const std::string& name) {
		if (!_core) {
			LOG_ERROR("Simulation core not initialised, cannot load robot");
			return;
		}
		if (!_rendererInitialised) {
			LOG_ERROR("Renderer not initialised, cannot load robot");
			return;
		}
		_core->loadRobot(name);
		if (!_core->hasRobot()) {
			LOG_ERROR("Failed to load robot: %s", name.c_str());
			return;
		}

		const auto& model = _core->robotSystem().model();
		auto world_src = [this]() -> const std::vector<mathlib::Mat4>& { 
			return _core->robotSystem().worldTransforms();
		};
		_systems.add(std::make_unique<MultiBodySystem>(model, world_src, _mesh_store, *_sim_renderer), _scene);
		_core->clearRobotPresentationDirty();
		LOG_INFO("Robot loaded: %s", model.name.c_str());
	}

	void SimulationManager::clearRobot() {
		_systems.clear_all(_scene); // clear all systems
        _scene.clear();          // drop all renderables
    }

	// --------------------------------------------------
	//				SIMULATION TICK & RENDER
	// --------------------------------------------------

	// Method to tick the simulation core, advancing the simulation state by the specified time step. This is typically called once per frame or at a fixed interval.
	void SimulationManager::tick(double dt) {
		_core->tick(dt);
	}

	void SimulationManager::setDisplaySize(int w, int h) {
		if (w <= 0.0f || h <= 0.0f) return;
		_displaySize = { w, h };
	}

	void SimulationManager::renderViewport(int w, int h) {
		if (!_rendererInitialised || w <= 0 || h <= 0) { return; }
		if (hasCompletedStudy()) {
			for (const auto& r : consumeCompletedStudy()) {
				LOG_INFO("Study completed: %s", r.tag.c_str());
			}
		}
		_systems.update_all(_scene);
		_renderer.render(_scene);
	}

	// --------------------------------------------------
	//					INPUT HANDLING
	// --------------------------------------------------
	void SimulationManager::pushCompletedStudies(std::vector<StudyResult>) { _hasCompletedStudy = true; }
    void SimulationManager::pushCompletedStudy(StudyResult)                { _hasCompletedStudy = true; }
    bool SimulationManager::hasCompletedStudy() const                      { return _hasCompletedStudy; }

    std::vector<StudyResult> SimulationManager::consumeCompletedStudy() {
        std::vector<StudyResult> copy;
        _hasCompletedStudy = false;
        return copy;
    }

    // ---------------- Input (stubbed until scene layer returns) ----------------

    void SimulationManager::processMovementKey(int, float) {}
    void SimulationManager::handleContinuousMovement(const std::unordered_set<eKeyCode>&, float) {}
    void SimulationManager::handleMouseLook(double, double, bool) {}
    void SimulationManager::onMouseWheel(double) {}
    void SimulationManager::resetMouseDelta() { _firstMouse = true; }

	// ---------------- Render settings ----------------

    void SimulationManager::applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r) {
        _settingsCurrent = s;
        _resCurrent = r;
        _settingsValid = true;
        // TODO: push to VulkanRenderer once it has a settings path
    }

    // ---------------- Objects ----------------

    void SimulationManager::setSelectedObject(scene::Object* obj) { _selectedObject = obj; }
    scene::Object* SimulationManager::getObject() { return _selectedObject; }
    std::vector<std::unique_ptr<scene::Object>>& SimulationManager::getObjects() { return _objects; }

    void SimulationManager::removeObject(scene::Object* obj) {
        if (!obj) { return; }
        if (_selectedObject == obj) { _selectedObject = nullptr; }
        std::erase_if(_objects, [obj](const std::unique_ptr<scene::Object>& p) { return p.get() == obj; });
    }

    // ---------------- Robots ----------------

    const bool SimulationManager::hasRobot() const { return false; }

    robots::RobotSystem& SimulationManager::robotSystem() { return _core->robotSystem(); }

    bool SimulationManager::followRobotJoint(const std::string&, const glm::vec3&) { return false; }

    // ---------------- Simulation control ----------------

	// Start the simulation
	void SimulationManager::startSimulation() {
		if (!hasRobot()) {
			LOG_WARN("Cannot start simulation: no robot loaded");
			return;
		}
		if (hasRobot()) {
			auto& rs = _core->robotSystem();
			load_robot(rs.robotName());
			return;
		}
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
	void SimulationManager::setActiveProgram(interpreter::IStoredProgram* program) { _core->setActiveProgram(program); }
	interpreter::IStoredProgram* SimulationManager::activeProgram() { return _core->activeProgram(); }
	const interpreter::IStoredProgram* SimulationManager::activeProgram() const { return _core->activeProgram(); }

	// Access the simulation core interface (non-const and const versions)
	core::ISimulationCore* SimulationManager::simCore() { return _core.get(); }
	const core::ISimulationCore* SimulationManager::simCore() const { return _core.get(); }

	// Set the integrator method for the current simulation run
	void SimulationManager::setIntegrationMethod(integration::eIntegrationMethod method) {
		_core->setIntegrationMethod(method);
	}
	const integration::eIntegrationMethod SimulationManager::integrationMethod() const {
		return _core->integrationMethod();
	}
	void SimulationManager::setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) {
		_core->setADIntegrationMethod(method);
	}
	const integration::eAutoDiffIntegrationMethod SimulationManager::autoDiffIntegrationMethod() const {
		return _core->autoDiffIntegrationMethod();
	}

	// This seems to be the better solution?
	static std::string replaceIntegratorInScript(const std::string& script, const std::string& methodName) {
		std::regex re(R"((?i)set\s*\(\s*integrator\s*,\s*([a-z0-9_]+)\s*\))"); // case-insensitive regex to match my DSL command -> set(integrator, method)
		std::string replacement = "set(integrator, " + methodName + ")";
		return std::regex_replace(script, re, replacement);
	}

	// Run a script to completion synchronously with a specific integrator
	bool SimulationManager::runScriptToCompletion(const std::string& scriptText, integration::eIntegrationMethod method) {
		if (!hasRobot()) { return false; }

		// Map method enum to string name
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45", "implicit_euler", "implicit_midpoint", "glrk2", "glrk3" };
		const std::string methodName = names[static_cast<int>(method)];

		// Replace the integrator method in the script text
		std::string modifiedScript = replaceIntegratorInScript(scriptText, methodName);

		// Create program and parser (bound to headless core)
		auto program = std::make_unique<interpreter::StoredProgram>(_core.get());
		//if (scene::Object* o = getObject()) program->setDefaultObject(o);
		auto parser = std::make_unique<interpreter::Parser>(program.get());

		// Parse the modified script and start the program
		parser->parse(modifiedScript);
		program->start();
		return _core->runScriptToCompletion(program.get(), method); // this will block until the script finishes
	}
}
