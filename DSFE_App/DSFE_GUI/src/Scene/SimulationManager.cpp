// DSFE_GUI SimulationManager.cpp
#include "Scene/Object.h"
#include "Scene/SimulationManager.h"
#include "Scene/SimulationCore.h"

#include <thread>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

#include "Manager/SimImplementation.h"
#include "SingleBodySystems/SingleBodySystem.h"
#include "Platform/KeyCode.h"

#include <filesystem>
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace fs = std::filesystem;

namespace gui {
	// Converts an Eigen 3D vector to a glm::vec3
	static glm::vec3 toGlm(const Vec3& v) {
		return glm::vec3(
			static_cast<float>(v.x()),
			static_cast<float>(v.y()),
			static_cast<float>(v.z())
		);
	}
	// Converts an Eigen quaternion to a glm::quat, taking into account the different ordering of components (w, x, y, z) vs (x, y, z, w)
	static glm::quat toGlm(const Quat& q) {
		return glm::quat(
			static_cast<float>(q.w()),
			static_cast<float>(q.x()),
			static_cast<float>(q.y()),
			static_cast<float>(q.z())
		); // (w, x, y, z)
	}

	// Converts a 3x3 Eigen matrix to a glm::mat3, taking into account the row-major to column-major conversion
	static glm::mat3 toGlm(const Mat3& m) {
		glm::mat3 g(1.0f);
		for (int c = 0; c < 3; ++c)
			for (int r = 0; r < 3; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g; // (3x3)
	}

	// Converts a 4x4 Eigen matrix to a glm::mat4, taking into account the row-major to column-major conversion
	static glm::mat4 toGlm(const Mat4& m) {
		glm::mat4 g(1.0f);
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g; // (4x4)
	}

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

	SimManager::SimManager() : _internalSize(1920, 1080), _displaySize(1.0f, 1.0f), _backgroundColour(0.18f, 0.18f, 0.20f),
		_backgroundAlpha(1.0f), _impl(std::make_unique<Impl>(*this)), _core(std::make_unique<core::SimulationCore>()),
		_studyRunner(std::make_unique<StudyRunner>(makeCoreFactory, std::thread::hardware_concurrency() > 1 ? std::thread::hardware_concurrency() - 1 : 1)) {
		_core->setRobotSystem(_impl->_robotSystem.get());
		_core->setTrajectoryManager(&_impl->_traj);
	}

	// Initialises OpenGL resources, including framebuffers, shaders, and IBL. Also picks an internal resolution preset based on the display size to balance quality and performance.
	void SimManager::initGL() {
		if (_glReady) return;
		_glReady = true;

		_impl->initGLResources(*this);

		InitShadowResource(_settingsCurrent.shadowMapRes);
		InitIBL();

		// Pick internal resolution preset based on display size
		render::ResolutionPreset bestPreset = render::ResolutionPreset::R_1080p;
		float dispH = _displaySize.y > 1.0f ? _displaySize.y : _internalSize.y;
		if (dispH >= 2000.0f) { bestPreset = render::ResolutionPreset::R_4K; }
		else if (dispH >= 1300.0f) { bestPreset = render::ResolutionPreset::R_1440p; }
		else if (dispH >= 900.0f) { bestPreset = render::ResolutionPreset::R_1080p; }
		else { bestPreset = render::ResolutionPreset::R_720p; }

		auto s = render::MakeSettings(bestPreset, render::QualityPreset::Medium);
		applyRenderProfile(s, bestPreset);
	}

	// Cleans up OpenGL resources
	SimManager::~SimManager() {
		// Clean up OpenGL resources
		if (_impl) {
			glDeleteFramebuffers(NUM_CASCADES, _impl->_cascadeFBO);
			glDeleteTextures(NUM_CASCADES, _impl->_cascadeDepth);
			if (_impl->_ssaoNoiseTex) glDeleteTextures(1, &_impl->_ssaoNoiseTex);
			if (_impl->_ssaoTex) glDeleteTextures(1, &_impl->_ssaoTex);
			if (_impl->_ssaoBlurTex) glDeleteTextures(1, &_impl->_ssaoBlurTex);
			if (_impl->_fullscreenVAO) glDeleteVertexArrays(1, &_impl->_fullscreenVAO);
			if (_impl->_worldGridVAO) glDeleteVertexArrays(1, &_impl->_worldGridVAO);
		}
		// Clean up scene objects and meshes if needed
		if (_impl && _impl->_mesh) { _impl->_mesh->clean(); }
	}

	// --------------------------------------------------
	// 			THREAD-SAFE SIMULATION RESULTS
	// --------------------------------------------------
	
	// Add a completed simulation run to the list in a thread-safe manner
	void SimManager::pushCompletedStudies(std::vector<StudyResult> results) {
		if (!_impl) { return; }
		std::lock_guard<std::mutex> lk(_impl->_completedRunsMutex);
		_impl->_completedRuns.insert(_impl->_completedRuns.end(), results.begin(), results.end());
		_hasCompletedStudy = true;
	}
	// Add a completed simulation run to the list in a thread-safe manner
	void SimManager::pushCompletedStudy(StudyResult result) {
		if (!_impl) { return; }
		std::lock_guard<std::mutex> lk(_impl->_completedRunsMutex);
		_impl->_completedRuns.push_back(std::move(result));
		_hasCompletedStudy = true;
	}

	bool SimManager::hasCompletedStudy() const { return _hasCompletedStudy; }

	// Retrieve and clear completed runs in a thread-safe manner
	std::vector<StudyResult> SimManager::consumeCompletedStudy() {
		std::vector<StudyResult> copy;
		if (!_impl) { return copy; }
		std::lock_guard<std::mutex> lk(_impl->_completedRunsMutex);
		copy = std::move(_impl->_completedRuns);
		_impl->_completedRuns.clear();
		_hasCompletedStudy = false;
		return copy;
	}

	void SimManager::tick(double dt) {
		_core->tick(dt);
		if (_core->robotPresentationDirty()) {
			loadRobot(_impl->_robotSystem->robotName());
			_core->clearRobotPresentationDirty();
		}
	}

	void SimManager::renderViewport(int w, int h) {
		if (!_glReady || !_impl) { return; }
		if (w <= 0 || h <= 0) { return; }
		if (hasCompletedStudy()) {
			auto results = consumeCompletedStudy();
			for (const auto& r : results) {
				LOG_INFO("Study completed: %s", r.tag.c_str());
				// TODO: update plots, telemetry graphs, UI panels here
			}
		}
		if (_impl->_robotSystem && hasRobot()) {
			_impl->_robotRenderer->applyTransforms(_impl->_robotSystem->model(), _impl->_robotSystem->worldTransforms());
		}
		auto& view = _impl->_views[static_cast<size_t>(_impl->activeView)];
		_impl->renderView(*this, view, w, h);
		glBindFramebuffer(GL_FRAMEBUFFER, _presentationFBO);
		glViewport(0, 0, w, h);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		_impl->_presentShader->use();
		_impl->_presentShader->setInt1(0, "screenTexture");
		
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, view.post->getTexture());
		glBindVertexArray(_impl->_fullscreenVAO);
		glDrawArrays(GL_TRIANGLES, 0, 3);
		glBindVertexArray(0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void SimManager::syncRobotToScene() {
		if (!hasRobot()) { return; }
		auto* rs = robotSystem();
		const auto& model = rs->model();
		const auto& T = rs->worldTransforms();

		for (size_t i = 0; i < model.links.size(); ++i) {
			const std::string& linkName = model.links[i].name;

			auto it = _impl->_linkToObjects.find(linkName);
			if (it == _impl->_linkToObjects.end()) continue;

			const glm::mat4& world = toGlm(T[i]);

			for (scene::Object* obj : it->second) {
				if (!obj) { continue; }
				glm::vec3 pos = glm::vec3(world[3]);
				glm::quat q = glm::quat_cast(world);
				obj->transform.position = pos;
				obj->transform.rotQ = q;
			}
		}
	}

	void SimManager::syncBodyToScene() {
		if (!hasBody()) { return; }
		auto* sys = singleBodySystem();
		const auto& body = sys->body();
		;
		auto it = _impl->_linkToObjects.find(body->name);
		if (it == _impl->_linkToObjects.end()) return;

		// For a single body, world transform are redundant, in fact the body->transform is already in world space so we can 
		auto& state = body->state;
		for (scene::Object* obj : it->second) {
			if (!obj) { continue; }
			glm::vec3 pos = toGlm(state.p);
			glm::quat q = toGlm(state.q);
			obj->transform.position = pos;
			obj->transform.rotQ = q;
		}
	}

	void SimManager::resize(int32_t width, int32_t height) {
		if (width <= 0 || height <= 0) return;
		_internalSize = { (float)width, (float)height };

		// Force per-view reallocation next frame
		for (auto& v : _impl->_views) {
			v.w = 0; v.h = 0;
			v.displayW = 0; v.displayH = 0;
		}

		//LOG_INFO("Resized SimManager INTERNAL RT to %dx%d", width, height);
	}

	void SimManager::setDisplaySize(int w, int h) {
		if (w <= 0.0f || h <= 0.0f) return;
		_displaySize = { w, h };
		for (auto& v : _impl->_views) {
			v.displayW = 0; v.displayH = 0; // Force per-view reallocation next frame
		}
	}

	// --------------------------------------------------
	//					INPUT HANDLING
	// --------------------------------------------------
	void gui::SimManager::processMovementKey(int key, float delta) {
		if (_impl->viewMode == Impl::ViewMode::Quad) { return; }// No keyboard movement in quad view
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		if (ctrlMode == ControlMode::Camera) { cam->processKeyboard(key, delta); }
		else if (ctrlMode == ControlMode::Object && _impl->_mesh) { /*idea is to add multiple angles to switch between!*/ }
	}

	void gui::SimManager::handleContinuousMovement(const std::unordered_set<eKeyCode>& pressedKeys, float dt) {
		if (_impl->viewMode == Impl::ViewMode::Quad) { return; } // No keyboard movement in quad view

		float kspd = 0.2f * dt; // base speed m/s

		if (pressedKeys.contains(eKeyCode::W)) { processMovementKey((int)eKeyCode::W, kspd); }
		if (pressedKeys.contains(eKeyCode::A)) { processMovementKey((int)eKeyCode::A, kspd); }
		if (pressedKeys.contains(eKeyCode::S)) { processMovementKey((int)eKeyCode::S, kspd); }
		if (pressedKeys.contains(eKeyCode::D)) { processMovementKey((int)eKeyCode::D, kspd); }
		if (pressedKeys.contains(eKeyCode::Space)) { processMovementKey((int)eKeyCode::Space, kspd); }
		if (pressedKeys.contains(eKeyCode::LShift)) { processMovementKey((int)eKeyCode::LShift, kspd); }
	}

// Handle mouse look (camera rotation) based on mouse movement. **OLD LOGIC FOR IMGUI AND GLFW**
	void gui::SimManager::handleMouseLook(double xpos, double ypos, bool mouseCaptured) {
		if (_impl->viewMode == Impl::ViewMode::Quad) { return; } // No mouse look in quad view
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		if (!mouseCaptured) {
			_lastMousePos = { static_cast<float>(xpos), static_cast<float>(ypos) };
			_firstMouse = true;
			return;
		}

		if (_firstMouse) {
			_lastMousePos = { static_cast<float>(xpos), static_cast<float>(ypos) };
			_firstMouse = false;
		}

		double xoffset = xpos;
		double yoffset = ypos;
		_lastMousePos = { static_cast<float>(xpos), static_cast<float>(ypos) };

		if (ctrlMode == ControlMode::Camera) { cam->processMouseMovement(static_cast<float>(xoffset), static_cast<float>(yoffset)); }
		else if (ctrlMode == ControlMode::Object && _impl->_selectedObject) { return; /*_impl->_selectedObject->onMouseMove(xpos, ypos, scene::eInputButton::Right);*/ }
	}
	void SimManager::onMouseWheel(double delta) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		auto* obj = _impl->_selectedObject;
		if (!_isHovered) return;

		if (ctrlMode == ControlMode::Camera) {
			// Always scroll the active view camera only
			cam->onMouseWheel(delta);
		}
		else if (ctrlMode == ControlMode::Object && _impl->_mesh) {
			obj->transform.position.z += (float)delta * 0.25f;
		}
	}

	void gui::SimManager::resetMouseDelta() { _firstMouse = true; }
}