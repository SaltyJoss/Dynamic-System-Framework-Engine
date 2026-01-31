
#include "pch.h"
#include "Scene/Object.h"
#include "Scene/SimulationManager.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <kinematics/Forward_Kinematics.h>

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtx/euler_angles.hpp>  
#include <imgui.h>

#include "Scene/Input.h"
#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/MeshLoader.h"
#include "Scene/Light.h"
#include "Scene/AxisOrientator.h"

#include "Physics/PhysicsSystem.h"
#include "Robots/RobotLoader.h"
#include "Robots/RobotModel.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"

#include "Interpreter/IStoredProgram.h"

#include "Rendering/SkyboxRenderer.h"
#include "Rendering/ShaderUtil.h"
#include "Rendering/OpenGLBufferManager.h"
#include "Rendering/IBL.h"
#include "Rendering/Texture.h"

#include <Platform/WindowManager.h>

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace gui {
	// --- PIMPL Implementation ---
	struct simManager::Impl {
		// View ID Alias
		using VID = gui::ViewID;

		// View Modes
		enum class ViewMode { Single, Quad };

		// Current View Mode
		ViewMode viewMode = ViewMode::Single;

		// Viewport Structure
		struct Viewport {
			std::unique_ptr<scene::Camera> cam;
			std::unique_ptr<render::OpenGLFrameBuffer> fb;
			std::unique_ptr<render::OpenGLFrameBuffer> post;
			// Cache size
			int w = 1, h = 1;
		};

		std::array<Viewport, (size_t)ViewID::COUNT> _views;
		VID activeView = VID::Manual;

		// Viewport & Render Targets
		std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer;
		std::unique_ptr<render::OpenGLFrameBuffer> _postBuffer;
		// Skybox & IBL
		std::unique_ptr<render::IBL> _ibl;
		std::unique_ptr<render::SkyboxRenderer> _skybox;

		// Post-Processing Shader
		std::unique_ptr<shaders::Shader> _postShader;
		std::shared_ptr<shaders::Shader> _shaderBasic;
		std::shared_ptr<shaders::Shader> _shaderLit;
		std::shared_ptr<shaders::Shader> _shaderPBR;

		// World Grid & Shadow Shaders
		std::unique_ptr<shaders::Shader> _worldGridShader;
		std::unique_ptr<shaders::Shader> _shadowShader;
		std::unique_ptr<shaders::Shader> _currentShader;
		shaders::Shader* currentShader;

		// Fullscreen Quad VAO
		GLuint _fullscreenVAO = 0;
		GLuint _worldGridVAO = 0;

		// Shadow Mapping (Cascaded)
		GLuint _cascadeFBO[simManager::NUM_CASCADES]{};
		GLuint _cascadeDepth[simManager::NUM_CASCADES]{};
		glm::mat4 _lightSpaceMatrixCascade[simManager::NUM_CASCADES] = {};

		// Scene Objects
		std::unique_ptr<scene::Camera> _camera;
		std::unique_ptr<scene::Light> _light;
		std::unique_ptr<AxisOrientator> _axisOrientator;

		scene::Object* _selectedObject = nullptr;
		scene::Object* _cameraFollowTarget = nullptr;

		std::shared_ptr<scene::Mesh> _mesh;
		std::vector<std::unique_ptr<scene::Object>> _objects;

		// Physics System
		std::unique_ptr<physics::PhysicsSystem> _physics;
		// Robot System
		std::unique_ptr<robots::RobotSystem> _robotSystem;

		// Trajectory Manager
		control::TrajectoryManager _traj;
		// Follow Target
		scene::Object* followTarget = nullptr;

		Impl(simManager& owner) {
			_frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
			_frameBuffer->createBuffers((int)owner._size.x, (int)owner._size.y, owner._settingsCurrent.msaaSamples);

			_postBuffer = std::make_unique<render::OpenGLFrameBuffer>();
			_postBuffer->createBuffers((int)owner._size.x, (int)owner._size.y, 1);

			_postShader = std::make_unique<shaders::Shader>();
			_postShader->load("Engine/assets/shaders/post.vert.glsl", "Engine/assets/shaders/post.frag.glsl");

			glGenVertexArrays(1, &_fullscreenVAO);

			// Lambda to create views
			auto makeView = [&](ViewID id, glm::vec3 pos, float fovDeg, glm::vec3 target, glm::vec3 upHint) {
				auto& v = _views[(size_t)id];

				// Create Framebuffers
				v.fb	= std::make_unique<render::OpenGLFrameBuffer>();
				v.post	= std::make_unique<render::OpenGLFrameBuffer>();

				// Create Camera
				v.w = (int)owner._size.x;
				v.h = (int)owner._size.y;

				// Position & FOV
				v.fb->createBuffers(v.w, v.h, owner._settingsCurrent.msaaSamples);
				v.post->createBuffers(v.w, v.h, 1);

				v.cam = std::make_unique<scene::Camera>(pos, fovDeg, (float)v.w / (float)v.h, 0.1f, 5000.0f);
				v.cam->setFocus(target);
				v.cam->updateViewMatrix();
			};

			glm::vec3 target(0.0f);

			// Perspective
			makeView(ViewID::Manual, { 0.0f, 0.5f, 1.0f }, 60.0f, target, { 0.0f, 1.0f, 0.0f });  // Default
			makeView(ViewID::Follow, { 0.0f, 0.25f, 3.0f }, 20.0f, target, { 0.0f, 1.0f, 0.0f }); // Follow
			// Ortho-ish
			makeView(ViewID::Top,	 { 0.0f, 3.0f, 0.0f }, 20.0f, target, { 0.0f, 0.0f, -1.0f }); // Top
			makeView(ViewID::Right, { 3.0f, 0.25f, 0.0f }, 20.0f, target,  { 0.0f, 1.0f, 0.0f }); // Right
			makeView(ViewID::Front, { 0.0f, 0.1f, 3.0f }, 20.0f, target, { 0.0f, -1.0f, 0.0f });  // Front

			// Now force their orientation using YOUR yaw/pitch system
			{
				// Top
				auto* camTop = _views[(size_t)ViewID::Top].cam.get();
				camTop->setFocus(target);
				camTop->setYaw(-glm::half_pi<float>());
				camTop->setPitch(-glm::half_pi<float>() + 0.001f);
				camTop->updateViewMatrix();

				// Right
				auto* camRight = _views[(size_t)ViewID::Right].cam.get();
				camRight->setFocus(target);
				camRight->setYaw(glm::pi<float>());
				camRight->setPitch(0.0f);
				camRight->updateViewMatrix();

				// Front
				auto* camFront = _views[(size_t)ViewID::Front].cam.get();
				camFront->setFocus(target);
				camFront->setYaw(-glm::half_pi<float>());
				camFront->setPitch(0.0f);
				camFront->updateViewMatrix();
			}

			// Shader Types A
			_shaderBasic = std::make_shared<shaders::Shader>();
			_shaderBasic->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_basic.frag.glsl");
			
			_shaderLit = std::make_shared<shaders::Shader>();
			_shaderLit->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_lit.frag.glsl");

			_shaderPBR = std::make_shared<shaders::Shader>();
			_shaderPBR->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_pbr.frag.glsl");

			currentShader = _shaderPBR.get();
			_skybox = std::make_unique<render::SkyboxRenderer>();

			// Shader Types B
			_worldGridShader = std::make_unique<shaders::Shader>();
			_worldGridShader->load("Engine/assets/shaders/world_grid.vert.glsl", "Engine/assets/shaders/world_grid.frag.glsl");

			_shadowShader = std::make_unique<shaders::Shader>();
			_shadowShader->load("Engine/assets/shaders/shadow_depth.vert.glsl", "Engine/assets/shaders/shadow_depth.frag.glsl");

			_light = std::make_unique<scene::Light>();
			_light->_isDirectional = true;

			_camera = std::make_unique<scene::Camera>(glm::vec3(0.0f, 0.25f, 1.0f), 60.0f, (float)owner._size.x / (float)owner._size.y, 0.1f, 5000.0f);
			_axisOrientator = std::make_unique<gui::AxisOrientator>();

			glGenVertexArrays(1, &_worldGridVAO);

			_mesh = std::make_shared<scene::Mesh>();
			_mesh->init();

			_physics = std::make_unique<physics::PhysicsSystem>();
			_robotSystem = std::make_unique<robots::RobotSystem>(_objects, [&owner](const std::string& path) { return owner.loadMeshReturn(path); });
		}

		void renderView(simManager& owner, Viewport& v, int vpW, int vpH) {
			vpW = std::max(1, vpW);
			vpH = std::max(1, vpH);

			// Resize if needed
			if (v.w != vpW || v.h != vpH) {
				v.w = vpW; v.h = vpH;

				v.fb->deleteBuffers();
				v.fb->createBuffers(v.w, v.h, owner._settingsCurrent.msaaSamples);

				v.post->deleteBuffers();
				v.post->createBuffers(v.w, v.h, 1);

				v.cam->setAspect((float)v.w / (float)v.h);
			}

			v.fb->bind();
			glViewport(0, 0, v.w, v.h);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(GL_TRUE);
			glDepthFunc(GL_LESS);

			glClearColor(owner._backgroundColour.r, owner._backgroundColour.g, owner._backgroundColour.b, owner._backgroundAlpha);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			if (owner.hasRobot()) { owner.getRobotSystem()->updateRobotKinematics(); }

			if (owner.skyboxEnabled) {
				glDepthMask(GL_FALSE);
				glDepthFunc(GL_LEQUAL);
				owner.SkyboxRender(v.cam.get());
				glDepthMask(GL_TRUE);
				glDepthFunc(GL_LESS);
			}

			owner.MeshRender(v.cam.get());
			if (owner._settingsCurrent.grid) { owner.WorldGridRender(v.cam.get()); }
			if (owner._settingsCurrent.axisOrientator) { _axisOrientator->render(v.cam->getViewMatrix(), owner._settingsCurrent.renderScale); }

			v.fb->unbind();

			// Post-Processing
			v.post->bind();
			glViewport(0, 0, v.w, v.h);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_BLEND);
			glClear(GL_COLOR_BUFFER_BIT);

			_postShader->use();
			_postShader->setInt1(0, "hdrScene");
			_postShader->setFlt1(owner._settingsCurrent.exposure, "exposure");
			_postShader->setFlt1(owner._settingsCurrent.whitePoint, "whitePoint");
			_postShader->setVec2(glm::vec2(v.w, v.h), "uRes");

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, v.fb->getTexture());

			glBindVertexArray(_fullscreenVAO);
			glDrawArrays(GL_TRIANGLES, 0, 3);
			glBindVertexArray(0);

			v.post->unbind();
		}
	};

	// ------

	// --------------------------------------------------
	//				CONSTRUCTOR & DESTRUCTOR
	// --------------------------------------------------

	simManager::simManager() : _size(3840, 2160), _backgroundColour(0.0f, 0.0f, 0.0f),
		_backgroundAlpha(1.0f), _impl(std::make_unique<Impl>(*this)) {
		_resSize = _size; // store initial size
	}

	void simManager::initGL() {
		if (_glReady) return;
		_glReady = true;

		InitShadowResource(_settingsCurrent.shadowMapRes);
		InitIBL();

		auto s = render::MakeSettings(render::ResolutionPreset::R_4K, render::QualityPreset::Ultra);
		applyRenderProfile(s, render::ResolutionPreset::R_4K);
	}

	simManager::~simManager() {
		if (_impl->_frameBuffer) _impl->_frameBuffer->deleteBuffers();
		if (_impl->_postBuffer) _impl->_postBuffer->deleteBuffers();
		if (_impl->_mesh) _impl->_mesh->clean();
	}

	// Helper to get the next ObjectID
	static inline scene::ObjectID next(scene::ObjectID id) { return static_cast<scene::ObjectID>(static_cast<std::uint32_t>(id) + 1); }

	// --------------------------------------------------
	//				    LIGHT & SKYBOX
	// --------------------------------------------------
	scene::Light* simManager::getLight() { return _impl->_light.get(); }

	void simManager::loadNewHDR(const std::string& path) {
		LOG_INFO("Loading new HDR: %s", path.c_str());
		D_INFO("Loading new HDR: %s", path.c_str());

		// Make sure IBL system exists
		if (!_impl->_ibl) {
			LOG_ERROR("Cannot load HDR because IBL system is not initialised.");
			D_FAIL("Cannot load HDR because IBL system is not initialised.");
			return;
		}

		_impl->_ibl->init(path); // rebuild envCubemap, irradiance, prefilter, brdfLUT
		D_SUCCESS("IBL rebuilt successfully.");

		// Update skybox
		_impl->_skybox->setEnvironmentTexture(_impl->_ibl->getEnvCubemap());

		_activeHDRPath = path;

		LOG_INFO("HDR updated successfully.");
		D_SUCCESS("Loaded HDR successfully.");
	}

	void simManager::loadNewHDR_UI(const std::string& path) {
		loadNewHDR(path);
		_hdrUserOverride = true;
	}

	void simManager::loadNewHDR_Preset(const std::string& path) {
		loadNewHDR(path);
		_hdrUserOverride = false;
	}

	// --------------------------------------------------
	//				CONTROL MODES & CAMERA
	// --------------------------------------------------
	
	// Get the active view camera
	scene::Camera* simManager::getCamera() { return _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get(); }
	// Reset the active view camera to default position
	void simManager::resetView() {
		auto& v = _impl->_views[static_cast<size_t>(_impl->activeView)];

		glm::vec3 pos = { 0.0f, 0.25f, 1.0f };
		float fov = 60.0f;

		switch (_impl->activeView) {
			case gui::ViewID::Top:   pos = { 0, 5, 0 }; fov = 20.0f; break;
			case gui::ViewID::Right: pos = { 5, 0, 0 }; fov = 20.0f; break;
			case gui::ViewID::Front: pos = { 0, 0, 5 }; fov = 20.0f; break;
			case gui::ViewID::Follow: fov = 20.0f; break;
			case gui::ViewID::Manual: fov = 20.0f; break;
			default: break;
		}

		float aspect = (float)std::max(1, v.w) / (float)std::max(1, v.h);
		v.cam = std::make_unique<scene::Camera>(pos, fov, aspect, 0.1f, 5000.0f);

		v.cam->setFocus(glm::vec3(0.0f));

		switch (_impl->activeView) {
		case gui::ViewID::Top:
			v.cam->setYaw(-glm::half_pi<float>());
			v.cam->setPitch(-glm::half_pi<float>() + 0.001f);
			break;
		case gui::ViewID::Right:
			v.cam->setYaw(glm::pi<float>());
			v.cam->setPitch(0.0f);
			break;
		case gui::ViewID::Front:
			v.cam->setYaw(-glm::half_pi<float>());
			v.cam->setPitch(0.0f);
			break;
		default:
			break;
		}

		v.cam->updateViewMatrix();
	}

	void simManager::attachCameraToObject(scene::Object* obj) {
		if (!obj) return;
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();

		_impl->_cameraFollowTarget = obj;

		glm::vec3 pos = obj->transform.position;
		glm::quat rot = obj->transform.rotQ;

		cam->startFollow(pos, rot, glm::vec3(0, 2, 5));
	}

	void simManager::detachCameraFromObject() {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		_impl->_cameraFollowTarget = nullptr;
		cam->clearFollow();
	}

// --------------------------------------------------
//			    MESH LOADING & GEOMETRY
// --------------------------------------------------
	void simManager::loadMesh(const std::string& filepath) {
		gui::MeshLoader loader;
		auto meshes = loader.load(filepath);

		if (meshes.empty()) {
			LOG_WARN("No meshes imported from %s", filepath.c_str());
			D_WARN("No meshes imported from %s", filepath.c_str());
			return;
		}

		// For now: spawn one Object per submesh
		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			obj->id = next(_nextObjectID);
			obj->source.filename = filepath;
			obj->name = m->getName().empty() ? "Object_" + std::to_string(scene::toUInt32(obj->id)) : m->getName();

			// initialise physics state
			obj->state.q = Quat(1.0, 0.0, 0.0, 0.0);
			obj->state.angularVelocity = Vec3::Zero();
			obj->state.linearVelocity = Vec3::Zero();
			obj->state.mass = 1.0;
			obj->state.damping = 0.0;
			obj->state.inertia = Mat3::Identity();
			obj->state.forces = Vec3::Zero();
			obj->state.torques = Vec3::Zero();

			_impl->_selectedObject = obj.get();
			_impl->_objects.push_back(std::move(obj));
		}

		LOG_INFO("Loaded %zu submeshes from %s", meshes.size(), filepath.c_str());
		D_INFO("Loaded %zu submeshes from %s", meshes.size(), filepath.c_str());
	}

	std::vector<scene::Object*> simManager::loadMeshReturn(const std::string& filepath) {
		gui::MeshLoader loader;
		auto meshes = loader.load(filepath);

		std::vector<scene::Object*> result;

		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			auto raw = obj.get();
			_impl->_objects.push_back(std::move(obj));
			result.push_back(raw);
		}
		return result;
	}

	void simManager::setMesh(std::shared_ptr<scene::Mesh> mesh) { _impl->_mesh = mesh; }
	std::shared_ptr<scene::Mesh> simManager::getMesh() { return _impl->_mesh; }

	void simManager::setSelectedObject(scene::Object* obj) { _impl->_selectedObject = obj; }
	void simManager::addObject(std::unique_ptr<scene::Object> obj) { _impl->_objects.push_back(std::move(obj)); } // Cache the unique_ptr

	void simManager::deleteObject(int index) {
		if (index < 0 || index >= _impl->_objects.size()) { return; }
		if (_impl->_selectedObject == _impl->_objects[index].get()) { _impl->_selectedObject = nullptr; }
		_impl->_objects.erase(_impl->_objects.begin() + index);
	}

	std::vector<std::unique_ptr<scene::Object>>& simManager::getObjects() { return _impl->_objects; }
	scene::Object* simManager::getObject() { return _impl->_selectedObject; }

	scene::Object* simManager::getObjectByID(scene::ObjectID id) {
		for (auto& obj : _impl->_objects) {
			if (obj && obj->id == id) {
				return obj.get();
			}
		}
		return nullptr;
	}

// --------------------------------------------------
//				RENDERING ENTRY POINTS
// --------------------------------------------------
	void simManager::render() {
		ImGuiIO& io = ImGui::GetIO();
		tick(io.DeltaTime);
		_fpsCounter.update();

		drawMainDockspace();
		drawViewportWindow();
	}

	// --- UI Elements ---

	// Main Dockspace with Menu Bar
	void simManager::drawMainDockspace() {
		ImGuiWindowFlags flags =
			ImGuiWindowFlags_NoDocking				|
			ImGuiWindowFlags_NoTitleBar				|
			ImGuiWindowFlags_NoCollapse				|
			ImGuiWindowFlags_NoResize				|
			ImGuiWindowFlags_NoMove					|
			ImGuiWindowFlags_NoBringToFrontOnFocus	|
			ImGuiWindowFlags_NoNavFocus;

		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(vp->Pos);
		ImGui::SetNextWindowSize(vp->Size);
		ImGui::SetNextWindowViewport(vp->ID);

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

		// Must be a window so DockSpace has somewhere to live
		ImGui::Begin("##MainDockspace", nullptr, flags);

		ImGui::PopStyleVar(2);

		beginSimManager("##MainDockspaceChild");

		ImGuiID dock_id = ImGui::GetID("MainDockspaceID");
		ImGui::DockSpace(dock_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

		endSimManager();

		ImGui::End();
	}

	// Viewport Window
	void simManager::drawViewportWindow() {
		ImGui::Begin("Viewport", nullptr,
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse);

		ImGuiIO& io = ImGui::GetIO();

		beginSimManager("##ViewportBody");

		// --- Tabs: Single / Quad ---
		if (ImGui::BeginTabBar("ViewportTabs", ImGuiTabBarFlags_None)) {

			const bool singleSelected = ImGui::BeginTabItem("Single");
			if (singleSelected) {
				_impl->viewMode = Impl::ViewMode::Single;
				ImGui::EndTabItem();
			}

			const bool quadSelected = ImGui::BeginTabItem("Quad");
			if (quadSelected) {
				_impl->viewMode = Impl::ViewMode::Quad;
				ImGui::EndTabItem();
			}

			ImGui::EndTabBar();
		}

		// Everything below tabs is render output
		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

		ImVec2 panel = ImGui::GetContentRegionAvail();

		// Pixel size (framebuffer coords)
		int vpW = (int)(panel.x * io.DisplayFramebufferScale.x);
		int vpH = (int)(panel.y * io.DisplayFramebufferScale.y);
		vpW = std::max(1, vpW);
		vpH = std::max(1, vpH);

		if (vpW != (int)_size.x || vpH != (int)_size.y) {
			resize(vpW, vpH);
		}

		// --- Render + Present ---
		if (_impl->viewMode == Impl::ViewMode::Quad) {
			int halfW = std::max(1, vpW / 2);
			int halfH = std::max(1, vpH / 2);

			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Top],	 halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Front],  halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Right],  halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Follow], halfW, halfH);

			// Stable 2x2 layout
			ImVec2 avail = ImGui::GetContentRegionAvail();
			ImVec2 cell = ImVec2(avail.x * 0.5f, avail.y * 0.5f);

			auto drawCell = [&](const char* childId, gui::ViewID id, bool sameLine) {
				if (sameLine) ImGui::SameLine();
				ImGui::BeginChild(childId, cell, false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

				auto& v = _impl->_views[(size_t)id];
				ImVec2 inner = ImGui::GetContentRegionAvail();

				ImGui::Image((ImTextureID)(intptr_t)v.post->getTexture(), inner, ImVec2(0, 1), ImVec2(1, 0));

				ImGui::EndChild();
				};

			drawCell("##Top", gui::ViewID::Top, false);
			drawCell("##Front", gui::ViewID::Front, true);
			drawCell("##Right", gui::ViewID::Right, false);
			drawCell("##Follow", gui::ViewID::Follow, true);
		}
		else {
			auto& v = _impl->_views[static_cast<size_t>(_impl->activeView)];
			_impl->renderView(*this, v, vpW, vpH);

			ImGui::Image((ImTextureID)(intptr_t)v.post->getTexture(), panel, ImVec2(0, 1), ImVec2(1, 0));
		}

		endSimManager();

		ImGui::End();
	}

	void simManager::resize(int32_t width, int32_t height) {
		// ignore zero sizes
		if (width == 0 || height == 0) { return; }
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		_size = glm::ivec2(width, height);

		if (_settingsValid) { rebuildRenderTargets(); }
		else {
			_impl->_frameBuffer->deleteBuffers();
			_impl->_frameBuffer->createBuffers(width, height, 1);

			_impl->_postBuffer->deleteBuffers();
			_impl->_postBuffer->createBuffers(width, height, 1);

			// update camera aspect ratio
			cam->setAspect((float)width / (float)height);
		}

		LOG_INFO("Resized simManager viewport to %dx%d", width, height);
	}

// --------------------------------------------------
//						PHYSICS
// --------------------------------------------------
	void gui::simManager::updatePhysics(double dt) {
		// Update each object's physics state
		for (auto& obj : _impl->_objects) {
			if (obj) { _impl->_physics->update(dt, obj.get()); }
		}
	}

	void simManager::stepFixed(double frame_dt) {
		//LOG_INFO("tick: simRunning=%d scriptRunning=%d activeProg=%p", (int)_simRunning, (int)_scriptRunning, (void*)_activeProgram);
		_accum += frame_dt;
		while (_accum >= _dt) {
			if (_scriptRunning && _activeProgram) {
				_activeProgram->step(_dt);

				const bool completed = _activeProgram->isCompleted();
				const bool stopped   = _activeProgram->isStopped();
				const bool faulted   = _activeProgram->isFaulted();

				if (completed || stopped || faulted) {
					D_FAIL("SCRIPT END: completed=%d stopped=%d faulted=%d (dt=%.6f simTime=%.3f)",
						(int)completed, (int)stopped, (int)faulted, _dt, _simTime);

					_scriptRunning = false;
					_activeProgram = nullptr;

					stopSimulation();
					D_DEBUG("Program execution completed.");
				}
			}
			else if (_scriptRunning && !_activeProgram) {
				D_FAIL("SCRIPT END: _scriptRunning=1 but _activeProgram=nullptr");
				_scriptRunning = false;
			}

			if (_simRunning) {
				_simTime += _dt;

				updatePhysics(_dt);
				if (hasRobot()) {
					_impl->_robotSystem->stepReference(_impl->_traj, _dt, _simTime);
					_impl->_robotSystem->step(_impl->_traj, _dt, _simTime);

					_telemetry.update(_simTime, *_impl->_robotSystem, &_impl->_traj, diagnostics::eTelemetryLevel::FULL);
				}
			}
			_accum -= _dt;
		}

		//D_DEBUG("Running state: %s", _scriptRunning ? "Running" : "Idle");
	}

	void simManager::startSimulation() {
		if (_simRunning) return; 
		D_INFO("starting simulation");
		_simTime = 0.0;
		_simRunning = true;
		DATA_CAPTURE_ENABLE(true);
	}

	void simManager::stopSimulation() {
		if (!_simRunning) return;
		D_INFO("stopping simulation");
		DATA_CAPTURE_ENABLE(false);
		_simRunning = false;
		_simTime = 0.0;
	}

	void simManager::tick(double frame_dt) { /*D_DEBUG("tick frame_dt=%.6f", frame_dt);*/ stepFixed(frame_dt); }
	physics::PhysicsSystem& simManager::getPhysicsSystem() { return *_impl->_physics; } // mutable
	const physics::PhysicsSystem& simManager::getPhysicsSystem() const { return *_impl->_physics; } // const

	control::TrajectoryManager& simManager::traj() { return _impl->_traj; }
	const control::TrajectoryManager& simManager::traj() const { return _impl->_traj; }


// --------------------------------------------------
//						ROBOTS
// --------------------------------------------------
	void simManager::loadRobot(const std::string& name) { if (_impl->_robotSystem) { _impl->_robotSystem->loadRobot(name); } }
	void simManager::setRobotLinkRotation(const std::string& linkName, float angle) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotLinkRotation(linkName, angle); } }
	void simManager::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootPose(pos, rot); } }
	void simManager::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootHome(pos, rot); } }
	void simManager::resetRobot() { if (_impl->_robotSystem) { _impl->_robotSystem->resetRobot(); } }
	void simManager::clearRobot() { if (_impl->_robotSystem) { _impl->_robotSystem->clearRobot(); } }
	bool simManager::hasRobot() const { return _impl->_robotSystem && _impl->_robotSystem->hasRobot(); }

	robots::RobotSystem* simManager::getRobotSystem() { return _impl->_robotSystem.get(); }
	const robots::RobotSystem* simManager::getRobotSystem() const { return _impl->_robotSystem.get(); }

// --------------------------------------------------
//			 INTERNAL REDNDERING PIPELINE
// --------------------------------------------------
	void simManager::InitShadowResource(int baseRes) {
		if (_shadowsInit) {
			glDeleteFramebuffers(simManager::NUM_CASCADES, _impl->_cascadeFBO);
			glDeleteTextures(simManager::NUM_CASCADES, _impl->_cascadeDepth);
		}

		glGenFramebuffers(simManager::NUM_CASCADES, _impl->_cascadeFBO);
		glGenTextures(simManager::NUM_CASCADES, _impl->_cascadeDepth);

		for (int i = 0; i < simManager::NUM_CASCADES; i++) {
			const int res = (i == 0) ? baseRes : (baseRes / 2); // 8192, 4096, 2048, 1024, 512, 256, 128

			glBindTexture(GL_TEXTURE_2D, _impl->_cascadeDepth[i]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, res, res, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			const float border[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

			glBindFramebuffer(GL_FRAMEBUFFER, _impl->_cascadeFBO[i]);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _impl->_cascadeDepth[i], 0);

			glDrawBuffer(GL_NONE);
			glReadBuffer(GL_NONE);
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		_shadowsInit = true;
	}

	void simManager::InitIBL() {
		_impl->_ibl = std::make_unique<render::IBL>();
		_impl->_ibl->init("Engine/assets/hdr/default_white.hdr");
	}

	void simManager::WorldGridRender(scene::Camera* cam) {
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glDepthMask(GL_FALSE);

		const int msaa = std::max(1, _settingsCurrent.msaaSamples);

		if (msaa > 1) {
			glDisable(GL_BLEND);
			glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
			glEnable(GL_MULTISAMPLE);

			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(-0.2f, -0.2f);
		}
		else {
			glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
			glDisable(GL_MULTISAMPLE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
		}

		_impl->_worldGridShader->use();
		_impl->_worldGridShader->setMat4(cam->getViewProjection(), "gVP");
		_impl->_worldGridShader->setVec3(cam->getPosition(), "gCameraWorldPos");
		_impl->_worldGridShader->setFlt1(_settingsCurrent.renderScale, "gRenderScale");

		glBindVertexArray(_impl->_worldGridVAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glBindVertexArray(0);

		glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);

		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glDepthFunc(GL_LESS);
	}

	void simManager::MeshRender(scene::Camera* cam) {
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);

		shaders::Shader* shader = nullptr;

		switch (currentShaderMode) {
			case ShaderMode::Basic:     
				shader = _impl->_shaderBasic.get();
				break;
			case ShaderMode::Lit:
				shader = _impl->_shaderLit.get();
				break;
			case ShaderMode::PBR:
				shader = _impl->_shaderPBR.get();
				break;
		}

		if (!shader) {
			LOG_ERROR("Shader is NULL after switch!");
			return;
		}

		shader->use();
		shader->setBool(false, "isFloor");

		// Only PBR know about cascades & those uniforms
		if (currentShaderMode == ShaderMode::PBR) {
			for (int i = 0; i < NUM_CASCADES; i++) {
				glActiveTexture(GL_TEXTURE5 + i);
				glBindTexture(GL_TEXTURE_2D, _impl->_cascadeDepth[i]);
				shader->setInt1(5 + i, "cascadeShadowMap[" + std::to_string(i) + "]");
				shader->setMat4(_impl->_lightSpaceMatrixCascade[i], "lightSpaceMatrix[" + std::to_string(i) + "]");
			}

			shader->setFlt2(_cascadeSplits[0], _cascadeSplits[1], "cascadeSplits");
		}

		// Camera / SunLight / light common to all mesh shaders
		cam->update(shader);
		_impl->_light->update(shader);

		for (auto& obj : _impl->_objects) {
			if (!obj || !obj->getMesh()) continue;

			if (_impl->_cameraFollowTarget == obj.get()) { cam->setFollowTarget( obj->transform.position, obj->transform.rotQ ); }

			glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;
			shader->setMat4(model, "model");
			shader->setBool(false, "isFloor");

			// Per-mode material uniforms
			switch (currentShaderMode)
			{
			case ShaderMode::Basic:
				// (IMPORTANT) mesh_basic.frag needs: uniform vec3 color;
				shader->setVec3(obj->getAlbedo(), "albedo");
				break;

			case ShaderMode::Lit:
				// (IMPORTANT) mesh_lit.frag needs: albedo, lightPosition, lightColour, lightIntensity, camPos
				shader->setVec3(obj->getAlbedo(), "albedo");
				shader->setVec3(_impl->_light->getPosition(), "lightPosition");
				shader->setFlt1(_impl->_light->getIntensity(), "lightIntensity");
				shader->setVec3(_impl->_light->getColour(), "lightColour");
				shader->setVec3(cam->getPosition(), "camPos");
				break;

			case ShaderMode::PBR:
				// (IMPORTANT) mesh_pbr.frag needs: albedo, metallic, roughness, ao, lightDirection, lightIntensity, lightColour, camPos
				shader->setVec3(obj->getAlbedo(), "albedo");
				shader->setFlt1(0.0f, "metallic");
				shader->setFlt1(0.5f, "roughness");
				shader->setFlt1(1.0f, "ao");

				shader->setVec3(glm::normalize(_impl->_light->getDirection()), "lightDirection");
				shader->setFlt1(_impl->_light->getIntensity(), "lightIntensity");
				shader->setVec3(_impl->_light->getColour(), "lightColour");
				shader->setVec3(cam->getPosition(), "camPos");

				shader->setInt1(0, "irradianceMap");
				shader->setInt1(1, "prefilterMap");
				shader->setInt1(2, "brdfLUT");

				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _impl->_ibl->getIrradianceMap());

				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _impl->_ibl->getPrefilterMap());

				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, _impl->_ibl->getBRDFLUT());

				LOG_INFO_ONCE("RadianceMap = %u, Prefilter = %u, BRDF = %u", _impl->_ibl->getIrradianceMap(), _impl->_ibl->getPrefilterMap(), _impl->_ibl->getBRDFLUT());
				break;
			}

			_impl->currentShader = shader; // for external access

			int loc = glGetUniformLocation(shader->getProgramID(), "albedo");
			LOG_INFO_ONCE("Lit Shader albedo uniform location = %d", loc);

			//obj->getMesh()->update(shader);
			obj->getMesh()->render();
		}
	}

	void simManager::ShadowPass(scene::Camera* cam) {
		float nearPlane = cam->getNear();
		float farPlane = cam->getFar();

		float cascadeNear[NUM_CASCADES]{};
		float cascadeFar[NUM_CASCADES]{};

		cascadeNear[0] = nearPlane;
		cascadeFar[0] = nearPlane + _cascadeSplits[0] * (farPlane);

		cascadeNear[1] = cascadeFar[0];
		cascadeFar[1] = nearPlane + _cascadeSplits[1] * (farPlane);

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(2.0f, 4.0f);

		for (int i = 0; i < NUM_CASCADES; i++) {
			_impl->_lightSpaceMatrixCascade[i] = LightSpaceMatrix(cam, cascadeNear[i], cascadeFar[i]);

			// set viewport to shadow map size
			int baseRes = _settingsCurrent.shadowMapRes;
			int res = (i == 0) ? baseRes : (baseRes / 2); // 4096, 2048, 1024, 512, 256, 128
			glViewport(0, 0, res, res);

			// render to cascade FBO
			glBindFramebuffer(GL_FRAMEBUFFER, _impl->_cascadeFBO[i]);
			glClear(GL_DEPTH_BUFFER_BIT);

			// render scene from light's point of view
			_impl->_shadowShader->use();
			_impl->_shadowShader->setMat4(_impl->_lightSpaceMatrixCascade[i], "lightSpaceMatrix");

			// main mesh
			for (auto& obj : _impl->_objects) {
				if (!obj || !obj->getMesh()) continue;

				glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;

				_impl->_shadowShader->setMat4(model, "model");
				obj->getMesh()->render();
			}
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, (int)_size.x, (int)_size.y);

		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	glm::mat4 simManager::LightSpaceMatrix(scene::Camera* cam, float nearPlane, float farPlane) {
		std::array<glm::vec4, 8> corners = cam->getFrustumCornersWorldSpace(nearPlane, farPlane);

		glm::vec3 lightDir = glm::normalize(_impl->_light->getDirection());

		// Fake camera position far along direction
		glm::vec3 lightPos = -lightDir * 50.0f;

		glm::mat4 lightView = glm::lookAt(
			lightPos,
			glm::vec3(0.0f),
			glm::vec3(0, 1, 0)
		);

		float minX = FLT_MAX, maxX = -FLT_MAX;
		float minY = FLT_MAX, maxY = -FLT_MAX;
		float minZ = FLT_MAX, maxZ = -FLT_MAX;

		for (auto& corner : corners) {
			glm::vec4 trf = lightView * glm::vec4(corner);
			minX = std::min(minX, trf.x);
			maxX = std::max(maxX, trf.x);
			minY = std::min(minY, trf.y);
			maxY = std::max(maxY, trf.y);
			minZ = std::min(minZ, trf.z);
			maxZ = std::max(maxZ, trf.z);
		}

		// Compute cascade center in light space
		glm::vec3 center = {
			0.5f * (minX + maxX),
			0.5f * (minY + maxY),
			0.5f * (minZ + maxZ)
		};

		// Cascade radius (half-size of the bounding sphere)
		float radius = glm::length(glm::vec3(maxX - minX, maxY - minY, 0.0f)) * 0.5f;

		int shadowMapResolution = _settingsCurrent.shadowMapRes;

		// The size of one texel in world-space
		float worldUnitsPerTexel = (radius * 2.0f) / shadowMapResolution;

		// Snap X and Y (Z never snapped)
		center.x = std::floor(center.x / worldUnitsPerTexel) * worldUnitsPerTexel;
		center.y = std::floor(center.y / worldUnitsPerTexel) * worldUnitsPerTexel;

		// Recompute min/max using snapped centre
		minX = center.x - radius;
		maxX = center.x + radius;
		minY = center.y - radius;
		maxY = center.y + radius;

		glm::mat4 lightProj = glm::ortho(minX, maxX, minY, maxY, minZ - 20.0f, maxZ + 20.0f);

		return lightProj * lightView;
	}

	void simManager::SkyboxRender(scene::Camera* cam) {
		glm::mat4 view = cam->getViewMatrix();
		glm::mat4 projection = cam->getProjection();

		_impl->_skybox->setEnvironmentTexture(_impl->_ibl->getEnvCubemap());
		_impl->_skybox->render(projection, view);
	}

	std::string simManager::getDefaultHDR() const { return "Engine/assets/hdr/default_white.hdr"; }	

	shaders::Shader* simManager::getActiveShader() const { return _impl->currentShader; }
	void simManager::applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r) { applyRenderProfile(s, r); }

	void simManager::applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r) {
		const bool first = !_settingsValid;

		const bool shadowResChanged = first || s.shadowMapRes != _settingsCurrent.shadowMapRes;

		const bool msaaChanged = first || (s.msaaSamples != _settingsCurrent.msaaSamples);
		const bool renderScaleChanged = first || (s.renderScale != _settingsCurrent.renderScale);

		if (shadowResChanged) {
			// Re-initialise shadow resources
			InitShadowResource(s.shadowMapRes);
			LOG_INFO("Shadow settings changed -> shadow resources re-initialised.");
			D_INFO("Shadow settings changed -> shadow resources re-initialised.");
		}

		_settingsCurrent = s;
		_resCurrent = r;

		if (msaaChanged || renderScaleChanged) { rebuildRenderTargets(); }

		LOG_INFO("Render settings applied: resPreset=%d shadowRes=%d msaa=%d renderScale=%.2f",  (int)r, _settingsCurrent.shadowMapRes, _settingsCurrent.msaaSamples, _settingsCurrent.renderScale);
		D_INFO("Render settings applied: resPreset=%d shadowRes=%d msaa=%d renderScale=%.2f", (int)r, _settingsCurrent.shadowMapRes, _settingsCurrent.msaaSamples, _settingsCurrent.renderScale);

		_settingsValid = true;
	}

	void simManager::rebuildRenderTargets() {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		const int vpW = (int)_size.x;
		const int vpH = (int)_size.y;

		const float scale = _settingsCurrent.renderScale;
		const int w = std::max(1, (int)std::lround(vpW * scale));
		const int h = std::max(1, (int)std::lround(vpH * scale));

		const int msaa = std::max(1, _settingsCurrent.msaaSamples);

		_impl->_frameBuffer->deleteBuffers();
		_impl->_frameBuffer->createBuffers(w, h, msaa);
		
		_impl->_postBuffer->deleteBuffers();
		_impl->_postBuffer->createBuffers(w, h, 1);

		cam->setAspect((float)vpW / (float)vpH);
		
		LOG_INFO("RenderTargets rebuilt: vp=%dx%d rt=%dx%d scale=%.2f msaa=%d", vpW, vpH, w, h, scale, msaa);
		D_INFO("RenderTargets rebuilt: vp=%dx%d rt=%dx%d scale=%.2f msaa=%d", vpW, vpH, w, h, scale, msaa);
	}

	void simManager::resetHDRToPreset() {
		_hdrUserOverride = false;
		const std::string hdr = getDefaultHDR();
		if (hdr != _activeHDRPath) { loadNewHDR(hdr); }
	}

	void simManager::reloadAllShaders()
	{
		_impl->_shaderBasic->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_basic.frag.glsl");
		_impl->_shaderLit->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_lit.frag.glsl");
		_impl->_shaderPBR->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_pbr.frag.glsl");

		LOG_INFO("All shaders reloaded from disk.");
		D_INFO_ONCE("All shaders reloaded from disk.");
	}

	void simManager::setLightColour(const glm::vec3& colour) { _impl->_light->_colour = colour; }

// --------------------------------------------------
//					INPUT HANDLING
// --------------------------------------------------
	void gui::simManager::processMovementKey(int key, float delta) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		if (ctrlMode == ControlMode::Camera) { cam->processKeyboard(key, delta); }
		else if (ctrlMode == ControlMode::Object && _impl->_mesh) { /*idea is to add multiple angles to switch between!*/ }
	}

	void gui::simManager::handleContinuousMovement(GLFWwindow* window, float dt) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		float kspd = 0.2f * dt; // base speed m/s

		if (scene::Input::IsKeyPressed(window, GLFW_KEY_W))				{ processMovementKey(GLFW_KEY_W,			kspd); } 
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_S))				{ processMovementKey(GLFW_KEY_S,			kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_A))				{ processMovementKey(GLFW_KEY_A,			kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_D))				{ processMovementKey(GLFW_KEY_D,			kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_SPACE))			{ processMovementKey(GLFW_KEY_SPACE,		kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_LEFT_SHIFT))	{ processMovementKey(GLFW_KEY_LEFT_SHIFT,	kspd); }
	}

	void gui::simManager::handleMouseLook(GLFWwindow* window, double xpos, double ypos) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		bool captured = false;
		if (win == static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window))) { captured = win->isMouseCaptured(); }

		if (!captured && !_isHovered) {
			_lastMousePos = { (float)xpos, (float)ypos };
			_firstMouse = true;
			return;
		}

		if (_firstMouse) {
			_lastMousePos = { (float)xpos, (float)ypos };
			_firstMouse = false;
		}

		double xoffset = xpos - _lastMousePos.x;
		double yoffset = _lastMousePos.y - ypos;
		_lastMousePos = { (float)xpos, (float)ypos };

		if (ctrlMode == ControlMode::Camera) { cam->processMouseMovement((float)xoffset, (float)yoffset); }
		else if (ctrlMode == ControlMode::Object && _impl->_selectedObject) { _impl->_selectedObject->onMouseMove(xpos, ypos, scene::eInputButton::Right); }
	}

	void simManager::onMouseMove(double x, double y, scene::eInputButton button) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		glm::vec2 pos2d{ x, y };
		glm::vec2 delta = pos2d - _lastMousePos;
		_lastMousePos = pos2d;

		if (!_isHovered) {
			cam->setCurrentPos2D(pos2d);
			_impl->_selectedObject->setLastMousePos(pos2d);
			return;
		}

		if (ctrlMode == ControlMode::Camera) { cam->onMouseMove(x, y, button); }
		else if (ctrlMode == ControlMode::Object && _impl->_selectedObject) { _impl->_selectedObject->onMouseMove(x, y, button); }
	}

	void simManager::onMouseWheel(double delta) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		auto* obj = _impl->_selectedObject;
		if (!_isHovered) return;

		if (ctrlMode == ControlMode::Camera) { cam->onMouseWheel(delta); }
		else if (ctrlMode == ControlMode::Object && _impl->_mesh) { obj->transform.position.z += (float)delta * 0.25f; }
	}

	void gui::simManager::resetMouseDelta() { _firstMouse = true; }

	// --- Helpers ---

	// Begin Control Panel Helper
	void simManager::beginSimManager(const char* id) {
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 8.0f));

		ImGui::BeginChild(id, ImVec2(0, 0), true, 
			ImGuiWindowFlags_AlwaysUseWindowPadding |
			ImGuiWindowFlags_NoScrollbar			|
			ImGuiWindowFlags_NoScrollWithMouse);
	}

	// End Control Panel Helper
	void simManager::endSimManager() {
		ImGui::EndChild();
		ImGui::PopStyleVar(4);
	}
}