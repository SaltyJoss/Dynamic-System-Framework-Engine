// DSFE_GUI SimulationManager.cpp
#include "Scene/Object.h"
#include "Simulation/SimulationManager.h"
#include "Simulation/SimulationRenderer.h"

#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"
#include "Robots/RobotModel.h"
#include "Robots/RobotSystem.h"
#include "Platform/ISimulationCore.h"

#include <thread>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

#include "SingleBodySystems/SingleBodySystem.h"
#include "Platform/KeyCode.h"

#include <filesystem>
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

	void SimulationManager::renderViewport(int w, int h) {
		if (!_rendererInitialised || w <= 0 || h <= 0) { return; }
		if (hasCompletedStudy()) {
			for (const auto& r : consumeCompletedStudy()) {
				LOG_INFO("Study completed: %s", r.tag.c_str());
			}
		}
		updateRobotTransforms();
		_renderer.render(_scene);
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

		auto model = _core->robotSystem().model();

		clearRobot();
		buildRobotVisuals(model);

		_core->clearRobotPresentationDirty();

		LOG_INFO("Robot loaded: %s", model.name.c_str());
	}

    void SimulationManager::buildRobotVisuals(const robots::RobotModel& model) {
        assets::MeshLoader loader;
        namespace fs = std::filesystem;

        for (const auto& link : model.links) {
            auto& renderables = _robot_binding.link_to_renderables[link.name];

            for (const auto& entry : link.visual.meshEntries) {
                fs::path full = paths::assets() / "objects" / "Robotic_Arm_Models" / entry.meshFile;

                auto meshes = loader.load(full.string());
                if (meshes.empty()) {
                    LOG_ERROR("buildRobotVisuals: no meshes in %s", full.string().c_str());
                    continue;
                }

                for (auto& mptr : meshes) {
                    scene::Mesh& src = *mptr;
                    if (src._vertices.empty()) { continue; }

                    // CPU + GPU registration, ids kept aligned (same pattern as loadMesh).
                    std::vector<uint32_t> indices(src._indices.begin(), src._indices.end());
                    const uint32_t cpu_id = _mesh_store.add(src);
                    const uint32_t gpu_id = _sim_renderer->upload(_mesh_store.get(cpu_id)->_vertices, indices);
                    if (cpu_id != gpu_id) {
                        LOG_ERROR("buildRobotVisuals: id mismatch %u vs %u", cpu_id, gpu_id);
                    }

					// Rest pose: scale only. Step F drives the real per-link world transforms
					// via the salvaged applyTransforms maths (toGlm + q_corr alignment).
					// glm::mat4 rest = glm::scale(glm::mat4(1.0f), glm::vec3(model.scale));
					// if (!model.baseFrameIsEngineAligned) {
					//  	glm::quat q_corr = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
					//  	rest = glm::mat4_cast(q_corr) * rest;
					// }
					const uint32_t r_idx = _scene.add_renderable(cpu_id, glm::mat4(1.0f));
                    renderables.push_back(r_idx);
                }
            }
        }
    }

	void SimulationManager::updateRobotTransforms() {
		if (!_core->hasRobot()) { return; }
		const auto& model = _core->robotSystem().model();
		const std::vector<mathlib::Mat4>& world = _core->robotSystem().worldTransforms();
		
		if (world.size() < model.links.size()) {
			LOG_ERROR("updateRobotTransforms: world transforms size (%zu) less than link count (%zu)", world.size(), model.links.size());
			return;
		}
		const bool is_aligned = model.baseFrameIsEngineAligned;
		for (size_t i = 0; i < model.links.size(); ++i) {
			const auto& link = model.links[i];
			auto it = _robot_binding.link_to_renderables.find(link.name);
			if (it == _robot_binding.link_to_renderables.end()) { continue; }
			glm::mat4 T = toGlm(world[i]);
			glm::vec3 pos = glm::vec3(T[3]); // translation
			glm::quat q = glm::quat_cast(T); // rotation
			//glm::quat q_rot = is_aligned ? q : q_corr * q; // apply correction if needed
			glm::mat4 M = toGlm(world[i]) * glm::scale(glm::mat4(1.0f), glm::vec3(model.scale));
			for (uint32_t r_idx : it->second) {
				_scene.set_transform(r_idx, M);
			}
		}
	}

	void SimulationManager::clearRobot() {
        _scene.clear();          // drop all renderables
        _robot_binding.clear();
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

    void SimulationManager::stopSimulation()      { _simRunning = false; }
    bool SimulationManager::isSimRunning() const  { return _simRunning; }
    double SimulationManager::simTime() const     { return _simTime; }
    void SimulationManager::setFixedDt(double dt) { if (dt > 0.0) { _fixedDt = dt; } }
    void SimulationManager::setTelemetryHz(double hz) { if (hz > 0.0) { _telemetryHz = hz; } }

    // ---------------- Scripting ----------------

    void SimulationManager::setScriptRunning(bool running) { _scriptRunning = running; }
    bool SimulationManager::isScriptRunning() const        { return _scriptRunning; }
    void SimulationManager::setLastScriptText(const std::string& text) { _lastScriptText = text; }

    void SimulationManager::setActiveProgram(interpreter::IStoredProgram* p) { _activeProgram = p; }
    interpreter::IStoredProgram* SimulationManager::activeProgram()          { return _activeProgram; }

    // ---------------- Core / telemetry ----------------

    core::ISimulationCore* SimulationManager::simCore()          { return _core.get(); }
    diagnostics::TelemetryRecorder& SimulationManager::telemetry() { return _telemetry; }

    // ---------------- Integration ----------------

    void SimulationManager::setIntegrationMethod(integration::eIntegrationMethod m) { _integrationMethod = m; }
    const integration::eIntegrationMethod SimulationManager::integrationMethod() const { return _integrationMethod; }

    void SimulationManager::setADIntegrationMethod(integration::eAutoDiffIntegrationMethod m) { _adIntegrationMethod = m; }
    const integration::eAutoDiffIntegrationMethod SimulationManager::autoDiffIntegrationMethod() const { return _adIntegrationMethod; }
}
