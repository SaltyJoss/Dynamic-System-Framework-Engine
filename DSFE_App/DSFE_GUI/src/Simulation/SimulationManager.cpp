// DSFE_GUI SimulationManager.cpp
#include "Scene/Object.h"
#include "Simulation/SimulationManager.h"

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

	// --------------------------------------------------
	//				CONSTRUCTOR & DESTRUCTOR
	// --------------------------------------------------

	SimulationManager::SimulationManager() : _internalSize(1920, 1080), _displaySize(1.0f, 1.0f), _backgroundColour(0.18f, 0.18f, 0.20f),
		_backgroundAlpha(1.0f), _core(CreateSimulationCore_v1(), CoreDeleter()),
		_studyRunner(std::make_unique<StudyRunner>(makeCoreFactory, std::thread::hardware_concurrency() > 1 ? std::thread::hardware_concurrency() - 1 : 1)) {
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
		_renderer.render();
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

    void SimulationManager::loadRobot(const std::string& name) {
        LOG_WARN("loadRobot('%s') is stubbed pending the Vulkan scene layer", name.c_str());
    }

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
