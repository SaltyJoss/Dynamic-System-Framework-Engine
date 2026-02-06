// SimulationManager.cpp
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
#include "Platform/Paths.h"

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
			int displayW = 1, displayH = 1;

			// Follow Target
			scene::Object* followTarget = nullptr;
			glm::vec3 followOffset = glm::vec3(0.0f, 0.25f, 1.0f); // tweak
			bool followEnabled = false;
		};

		std::array<Viewport, (size_t)ViewID::COUNT> _views;
		VID activeView = VID::Manual;

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
		bool eeFollowBound = false;
		scene::Object* eeObject = nullptr;

		// Trajectory Manager
		control::TrajectoryManager _traj;
		// Follow Target
		scene::Object* followTarget = nullptr;

		Impl(simManager& owner) {
			_postShader = std::make_unique<shaders::Shader>();
			_postShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "post.frag.glsl").string());

			glGenVertexArrays(1, &_fullscreenVAO);

			// Lambda to create views
			auto makeView = [&](ViewID id, glm::vec3 pos, float fovDeg, glm::vec3 target, glm::vec3 upHint) {
				auto& v = _views[(size_t)id];

				// INTERNAL render target size (scene render)
				const int rtW = std::max(1, (int)owner._internalSize.x);
				const int rtH = std::max(1, (int)owner._internalSize.y);

				// DISPLAY size (final post-process target)
				const int dispW = std::max(1, (int)owner._displaySize.x);
				const int dispH = std::max(1, (int)owner._displaySize.y);
				const int postW = (dispW > 1 && dispH > 1) ? dispW : rtW;
				const int postH = (dispW > 1 && dispH > 1) ? dispH : rtH;

				// Cache sizes
				v.w = rtW;
				v.h = rtH;
				v.displayW = postW;
				v.displayH = postH;

				// Framebuffers
				v.fb = std::make_unique<render::OpenGLFrameBuffer>();
				v.fb->createBuffers(rtW, rtH, std::max(1, owner._settingsCurrent.msaaSamples));

				v.post = std::make_unique<render::OpenGLFrameBuffer>();
				v.post->createBuffers(postW, postH, 1);

				// Camera aspect should match DISPLAY (what you're presenting in ImGui)
				v.cam = std::make_unique<scene::Camera>(pos, fovDeg, (float)postW / (float)postH, 0.1f, 5000.0f);
				v.cam->setFocus(target);
				v.cam->updateViewMatrix();
			};

			glm::vec3 target(0.0f);

			// Perspective
			makeView(ViewID::Manual, { 0.0f, 0.5f, 1.0f },  60.0f, target, { 0.0f, 1.0f, 0.0f });  // Default
			makeView(ViewID::Follow, { 3.0f, 0.25f, 0.0f }, 20.0f, target, { 0.0f, 1.0f, 0.0f }); // Follow
			// Ortho-ish
			makeView(ViewID::Top,   { 0.0f, 2.0f, 0.0f },  20.0f, target, { 0.0f, 0.0f, -1.0f }); // Top
			makeView(ViewID::Right, { 3.0f, 0.25f, 0.0f }, 20.0f, target, { 0.0f, 1.0f, 0.0f }); // Right
			makeView(ViewID::Front, { 0.0f, 0.25f, 3.0f },  20.0f, target, { 0.0f, -1.0f, 0.0f }); // Front

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

				// Follow
				auto* camFollow = _views[(size_t)ViewID::Follow].cam.get();
				camFollow->setFocus(target);
				camFollow->setYaw(glm::pi<float>());
				camFollow->setPitch(0.0f);
				camFollow->updateViewMatrix();
			}

			// Shader Types A
			_shaderBasic = std::make_shared<shaders::Shader>();
			_shaderBasic->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_basic.frag.glsl").string());

			_shaderLit = std::make_shared<shaders::Shader>();
			_shaderLit->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_lit.frag.glsl").string());

			_shaderPBR = std::make_shared<shaders::Shader>();
			_shaderPBR->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_pbr.frag.glsl").string());

			currentShader = _shaderPBR.get();
			_skybox = std::make_unique<render::SkyboxRenderer>();

			// Shader Types B
			_worldGridShader = std::make_unique<shaders::Shader>();
			_worldGridShader->load((paths::assets() / "shaders" / "world_grid.vert.glsl").string(), (paths::assets() / "shaders" / "world_grid.frag.glsl").string());

			_shadowShader = std::make_unique<shaders::Shader>();
			_shadowShader->load((paths::assets() / "shaders" / "shadow_depth.vert.glsl").string(), (paths::assets() / "shaders" / "shadow_depth.frag.glsl").string());

			_light = std::make_unique<scene::Light>();
			_light->_isDirectional = true;

			_camera = std::make_unique<scene::Camera>(glm::vec3(0.0f, 0.25f, 1.0f), 60.0f, (float)owner._internalSize.x / (float)owner._internalSize.y, 0.1f, 5000.0f);
			_axisOrientator = std::make_unique<gui::AxisOrientator>();

			glGenVertexArrays(1, &_worldGridVAO);

			_mesh = std::make_shared<scene::Mesh>();
			_mesh->init();

			_physics = std::make_unique<physics::PhysicsSystem>();
			_robotSystem = std::make_unique<robots::RobotSystem>(_objects, [&owner](const std::string& path) { return owner.loadMeshReturn(path); });
		}

		void renderView(simManager& owner, Viewport& v, int displayW, int displayH) {
			displayW = std::max(1, displayW);
			displayH = std::max(1, displayH);

			glm::ivec2 rt(std::max(1, (int)owner._internalSize.x), std::max(1, (int)owner._internalSize.y));

			if (owner._impl->viewMode == Impl::ViewMode::Quad) {
				rt.x = std::max(1, rt.x / 2);
				rt.y = std::max(1, rt.y / 2);
			}
			const int rtW = std::max(1, (int)rt.x);
			const int rtH = std::max(1, (int)rt.y);

			LOG_INFO_ONCE("Rendering Viewport: RT Size = %dx%d, Display Size = %dx%d", rtW, rtH, displayW, displayH);

			// Calculate internal scale for grid rendering
			const float internalScaleX = (float)rtW / (float)displayW;
			const float internalScaleY = (float)rtH / (float)displayH;
			const float internalScale = std::max(internalScaleX, internalScaleY);

			// Resize only when internal RT changes OR display changes (post buffer)
			const bool rtChanged = (v.w != rtW) || (v.h != rtH);
			const bool displayChanged = (v.displayW != displayW) || (v.displayH != displayH);
			const bool msaaChanged = false;

			if (rtChanged || displayChanged || msaaChanged) {
				// Store internal RT size
				v.w = rtW;
				v.h = rtH;

				// Store display size
				v.displayW = displayW;
				v.displayH = displayH;

				// Adjust MSAA based on internal RT size
				int msaa = std::max(1, owner._settingsCurrent.msaaSamples);

				// Scene framebuffer: INTERNAL resolution (rtW x rtH)
				v.fb->deleteBuffers();
				v.fb->createBuffers(rtW, rtH, msaa);

				// Post-processing framebuffer: DISPLAY resolution (displayW x displayH)
				v.post->deleteBuffers();
				v.post->createBuffers(displayW, displayH, 1);

				// Camera aspect must match DISPLAY aspect
				v.cam->setAspect((float)displayW / (float)displayH);
			}

			v.fb->bind();
			glViewport(0, 0, v.w, v.h);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(GL_TRUE);
			glDepthFunc(GL_LESS);

			glClearColor(owner._backgroundColour.r, owner._backgroundColour.g, owner._backgroundColour.b, owner._backgroundAlpha);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			if (owner._settingsCurrent.msaaSamples > 1) {
				glEnable(GL_MULTISAMPLE);
			}
			else {
				glDisable(GL_MULTISAMPLE);
			}

			if (owner.hasRobot()) { owner.getRobotSystem()->updateRobotKinematics(); }

			// Update Follow Target (Doesnt deref pointer until used - hopefully fixes previous crashes)
			if (&v == &_views[(size_t)gui::ViewID::Follow]) {
				v.followTarget = _selectedObject;
				v.followEnabled = (v.followTarget != nullptr);

				if (v.followEnabled && v.cam) {
					auto* mesh = v.followTarget->getMesh();
					if (!mesh) {
						v.followTarget = nullptr;
						v.followEnabled = false;
					}
					else {
						glm::mat4 M = v.followTarget->transform.toMatrix() * mesh->localTransform;
						glm::vec3 worldPos = glm::vec3(M[3]);
						v.cam->setFollowTarget(worldPos, v.followTarget->transform.rotQ);
					}
				}
			}


			if (owner.skyboxEnabled) {
				glDepthMask(GL_FALSE);
				glDepthFunc(GL_LEQUAL);
				owner.SkyboxRender(v.cam.get());
				glDepthMask(GL_TRUE);
				glDepthFunc(GL_LESS);
			}

			GLint sampleBuffers = 0, samples = 0;
			glGetIntegerv(GL_SAMPLE_BUFFERS, &sampleBuffers);
			glGetIntegerv(GL_SAMPLES, &samples);
			LOG_INFO_ONCE("FB MSAA state: GL_SAMPLE_BUFFERS=%d GL_SAMPLES=%d", sampleBuffers, samples);

			owner.MeshRender(v.cam.get());
			if (owner._settingsCurrent.grid) { owner.WorldGridRender(v.cam.get()); }

			v.fb->unbind();

			// Only valid if you allocated mip levels for _texID (via glTexStorage2D)
			glBindTexture(GL_TEXTURE_2D, v.fb->getTexture());
			glGenerateMipmap(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, 0);

			// Post-Processing
			v.post->bind();
			glDisable(GL_MULTISAMPLE);
			glViewport(0, 0, v.displayW, v.displayH);

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
			if (owner._settingsCurrent.axisOrientator) { _axisOrientator->render(v.cam->getViewMatrix()); }
			glBindVertexArray(0);

			v.post->unbind();
		}

		static bool icontains(const std::string& s, const char* sub) {
			auto it = std::search(
				s.begin(), s.end(),
				sub, sub + std::strlen(sub),
				[](char a, char b) { return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); }
			);
			return it != s.end();
		}

		scene::Object* findEndEffectorFromRange(size_t startIdx) {
			for (size_t i = startIdx; i < _objects.size(); ++i) {
				scene::Object* o = _objects[i].get();
				if (!o) continue;

				const std::string& n = o->name;
				if (icontains(n, "end") || icontains(n, "eff") || icontains(n, "ee") || icontains(n, "tool") || icontains(n, "tcp") || icontains(n, "gripper")) {
					return o;
				}
			}

			// Fallback: last object added (usually the last link)
			if (_objects.size() > startIdx) return _objects.back().get();
			return nullptr;
		}
	};

	// --------------------------------------------------
	//				CONSTRUCTOR & DESTRUCTOR
	// --------------------------------------------------

	simManager::simManager() : _internalSize(3840, 2160), _displaySize(1.0f, 1.0f), _backgroundColour(0.0f, 0.0f, 0.0f),
		_backgroundAlpha(1.0f), _impl(std::make_unique<Impl>(*this)) {
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
		//if (_impl->_frameBuffer) _impl->_frameBuffer->deleteBuffers();
		//if (_impl->_postBuffer) _impl->_postBuffer->deleteBuffers();
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

	// Set the follow target for a specific view
	void simManager::setViewFollowTarget(ViewID view, scene::Object* obj, const glm::vec3& offset) {
		if (view < ViewID::Manual || view >= ViewID::COUNT) { return; }
		auto& v = _impl->_views[static_cast<size_t>(view)];
		v.followTarget = obj;
		v.followOffset = offset;
		v.followEnabled = (obj != nullptr);

		if (v.followEnabled && v.cam && obj) {
			v.cam->startFollow(obj->transform.position, obj->transform.rotQ, offset);
		}
	}

	// Clear the follow target for a specific view
	void simManager::clearViewFollowTarget(ViewID view) {
		if (view < ViewID::Manual || view >= ViewID::COUNT) { return; }
		auto& v = _impl->_views[static_cast<size_t>(view)];
		v.followTarget = nullptr;
		v.followEnabled = false;
		if (v.cam) { v.cam->clearFollow(); }
	}

	bool simManager::setViewFollowRobotJoint(ViewID view, const std::string& jointName, const glm::vec3& offset) {
		if (!hasRobot()) {
			LOG_WARN("setViewFollowRobotJoint: no robot loaded");
			return false;
		}

		robots::RobotSystem* rs = getRobotSystem();
		if (!rs) return false;

		auto& joints = rs->joints();
		auto& links = rs->links();

		// 1) Find joint by name
		const robots::RobotJoint* jPtr = nullptr;
		for (auto& j : joints) {
			if (j.name == jointName) { jPtr = &j; break; }
		}
		if (!jPtr) {
			LOG_WARN("setViewFollowRobotJoint: joint not found: %s", jointName.c_str());
			return false;
		}

		// 2) Find child link -> attached object
		scene::Object* targetObj = nullptr;
		for (auto& l : links) {
			if (l.name == jPtr->child) {
				targetObj = l.attachedObject; // this is the key
				break;
			}
		}

		if (!targetObj) {
			LOG_WARN("setViewFollowRobotJoint: no attached object for joint=%s child=%s",
				jointName.c_str(), jPtr->child.c_str());
			return false;
		}

		// 3) Bind the view follow target
		setViewFollowTarget(view, targetObj, offset);

		LOG_INFO("Follow view=%d bound to joint='%s' -> child='%s' -> obj='%s'",
			(int)view, jointName.c_str(), jPtr->child.c_str(), targetObj->name.c_str());

		return true;
	}

	bool simManager::followRobotJoint(const std::string& jointName, const glm::vec3& offset) {
		return setViewFollowRobotJoint(gui::ViewID::Follow, jointName, offset);
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
			ImGuiWindowFlags_NoDocking |
			ImGuiWindowFlags_NoTitleBar |
			ImGuiWindowFlags_NoCollapse |
			ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoBringToFrontOnFocus |
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
		int vpW = (int)(panel.x);
		int vpH = (int)(panel.y);
		vpW = std::max(1, vpW);
		vpH = std::max(1, vpH);

		// Update DISPLAY size only
		if ((int)_displaySize.x != vpW || (int)_displaySize.y != vpH) {
			_displaySize = { (float)vpW, (float)vpH };

			// invalidate only display cached sizes so post buffers resize
			for (auto& v : _impl->_views) {
				v.displayW = 0;
				v.displayH = 0;
			}
			LOG_INFO("Viewport display size updated to %dx%d", vpW, vpH);
		}

		// --- Render + Present ---
		if (_impl->viewMode == Impl::ViewMode::Quad) {
			int halfW = std::max(1, vpW / 2);
			int halfH = std::max(1, vpH / 2);

			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Top], halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Front], halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Right], halfW, halfH);
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
		if (width <= 0 || height <= 0) return;
		_internalSize = { (float)width, (float)height };

		// Force per-view reallocation next frame
		for (auto& v : _impl->_views) {
			v.w = 0; v.h = 0;
			v.displayW = 0; v.displayH = 0;
		}

		LOG_INFO("Resized simManager INTERNAL RT to %dx%d", width, height);
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

	// --------------------------------------------------
	//						ROBOTS
	// --------------------------------------------------
	void simManager::loadRobot(const std::string& name) {
		setSelectedObject(nullptr);
		detachCameraFromObject();
		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;

		if (!_impl->_robotSystem) return;

		const size_t startIdx = _impl->_objects.size();
		_impl->_robotSystem->loadRobot(name);

		scene::Object* ee = _impl->findEndEffectorFromRange(startIdx);
		if (ee) {
			_impl->eeObject = ee;
			setViewFollowTarget(gui::ViewID::Follow, ee, glm::vec3(0.0f, 0.2f, 0.6f));
			_impl->eeFollowBound = true;

			LOG_INFO("Follow view bound to end-effector candidate: %s", ee->name.c_str());
		}
		else {
			LOG_WARN("Could not find end-effector object to follow.");
		}

		_impl->_robotSystem->setDefaultPoseDeg({ -45.0f, 33.5f, -42.5f, 12.5f, 0.0f, 0.0f });
	}
	void simManager::setRobotLinkRotation(const std::string& linkName, float angle) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotLinkRotation(linkName, angle); } }
	void simManager::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootPose(pos, rot); } }
	void simManager::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) { if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootHome(pos, rot); } }
	void simManager::resetRobot() { if (_impl->_robotSystem) { _impl->_robotSystem->resetRobot(); } }
	void simManager::clearRobot() {
		setSelectedObject(nullptr); // deselect any selected object
		if (_impl->_robotSystem) { _impl->_robotSystem->clearRobot(); }

		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;
	}
	bool simManager::hasRobot() const { return _impl->_robotSystem && _impl->_robotSystem->hasRobot(); }

	robots::RobotSystem* simManager::getRobotSystem() { return _impl->_robotSystem.get(); }
	const robots::RobotSystem* simManager::getRobotSystem() const { return _impl->_robotSystem.get(); }

	// --------------------------------------------------
	//					SIMULATION LOOP
	// --------------------------------------------------

	void simManager::stepFixed(double frame_dt) {
		//LOG_INFO("tick: simRunning=%d scriptRunning=%d activeProg=%p", (int)_simRunning, (int)_scriptRunning, (void*)_activeProgram);
		_accum += frame_dt;
		while (_accum >= _dt) {
			if (_scriptRunning && _activeProgram) {
				_activeProgram->step(_dt);

				const bool completed = _activeProgram->isCompleted();
				const bool stopped = _activeProgram->isStopped();
				const bool faulted = _activeProgram->isFaulted();

				if (completed) {
					D_SUCCESS("SCRIPT END: completed=%d (dt=%.6f simTime=%.3f)",
						(int)completed, _dt, _simTime);

					_scriptRunning = false;
					_activeProgram = nullptr;

					stopSimulation();
					D_RUNTIME("Program execution completed.");
				}
				else if (stopped || faulted) {
					D_FAIL("SCRIPT END: stopped=%d faulted=%d (dt=%.6f simTime=%.3f)",
						(int)stopped, (int)faulted, _dt, _simTime);

					_scriptRunning = false;
					_activeProgram = nullptr;

					stopSimulation();
					D_RUNTIME("Program execution completed.");
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
					if (!_telemetryBegun) {
						_telemetry.beginRun(_simTime, 60.0, 120.0);
						_telemetryBegun = true;
					}
					_telemetry.update(_simTime, *_impl->_robotSystem, &_impl->_traj, diagnostics::eTelemetryLevel::FULL);
				}
			}
			_accum -= _dt;
		}
	}

	void simManager::startSimulation() {
		if (_simRunning) return;
		telemetry().clear();
		D_RUNTIME("starting simulation");
		_simTime = 0.0;
		_simRunning = true;
		_telemetryBegun = true;
		DATA_CAPTURE_ENABLE(true);
	}

	void simManager::stopSimulation() {
		if (!_simRunning) return;
		D_RUNTIME("stopping simulation");
		DATA_CAPTURE_ENABLE(false);
		_simRunning = false;
		_telemetryBegun = false;
	}

	void simManager::tick(double frame_dt) { /*D_DEBUG("tick frame_dt=%.6f", frame_dt);*/ stepFixed(frame_dt); }
	physics::PhysicsSystem& simManager::getPhysicsSystem() { return *_impl->_physics; } // mutable
	const physics::PhysicsSystem& simManager::getPhysicsSystem() const { return *_impl->_physics; } // const

	control::TrajectoryManager& simManager::traj() { return _impl->_traj; }
	const control::TrajectoryManager& simManager::traj() const { return _impl->_traj; }

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
		_impl->_ibl->init((paths::assets() / "hdr" / "default_white.hdr").string());
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

			// Convert fractional splits (0..1 of far) into world-space distances
			const float nearPlane = cam->getNear();
			const float farPlane = cam->getFar();

			const float splitFrac0 = _cascadeSplits[0];
			const float splitFrac1 = _cascadeSplits[1];

			const float splitDist0 = nearPlane + splitFrac0 * farPlane;
			const float splitDist1 = nearPlane + splitFrac1 * farPlane;

			shader->setFlt2(splitDist0, splitDist1, "cascadeSplits");
		}

		// Camera / SunLight / light common to all mesh shaders
		cam->update(shader);
		_impl->_light->update(shader);

		for (auto& obj : _impl->_objects) {
			if (!obj || !obj->getMesh()) continue;

			if (_impl->_cameraFollowTarget == obj.get()) { cam->setFollowTarget(obj->transform.position, obj->transform.rotQ); }

			glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;
			shader->setMat4(model, "model");
			shader->setBool(false, "isFloor");

			// Per-mode material uniforms
			switch (currentShaderMode) {
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

					break;
			}

			_impl->currentShader = shader; // for external access
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
		glViewport(0, 0, (int)_internalSize.x, (int)_internalSize.y);

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

	std::string simManager::getDefaultHDR() const { return (paths::assets() / "hdr"/ "default_white.hdr").string(); }

	shaders::Shader* simManager::getActiveShader() const { return _impl->currentShader; }
	void simManager::applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r) { applyRenderProfile(s, r); }

	void simManager::applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r) {
		const bool first = !_settingsValid;

		const bool shadowResChanged = first || (s.shadowMapRes != _settingsCurrent.shadowMapRes);
		const bool msaaChanged = first || (s.msaaSamples != _settingsCurrent.msaaSamples);
		const bool renderScaleChanged = first || (s.renderScale != _settingsCurrent.renderScale);
		const bool presetChanged = first || (r != _resCurrent);

		if (shadowResChanged) {
			InitShadowResource(s.shadowMapRes);
		}

		_settingsCurrent = s;
		_resCurrent = r;

		if (msaaChanged || renderScaleChanged || presetChanged) {
			for (auto& v : _impl->_views) { v.w = v.h = 0; v.displayW = v.displayH = 0; }
		}

		glm::vec2 px = getPresetResolutionPx();
		_internalSize = px;
		for (auto& v : _impl->_views) { v.w = v.h = 0; } // internal invalidation

		LOG_INFO("Render settings applied: resPreset=%d shadowRes=%d msaa=%d renderScale=%.2f", (int)r, _settingsCurrent.shadowMapRes, _settingsCurrent.msaaSamples, _settingsCurrent.renderScale);
		D_RUNTIME("Render settings applied: resPreset=%d shadowRes=%d msaa=%d renderScale=%.2f", (int)r, _settingsCurrent.shadowMapRes, _settingsCurrent.msaaSamples, _settingsCurrent.renderScale);

		_settingsValid = true;
	}

	glm::vec2 simManager::getPresetResolutionPx() const {
		switch (_resCurrent) {
			case render::ResolutionPreset::R_720p:  return { 1280, 720 };
			case render::ResolutionPreset::R_1080p: return { 1920, 1080 };
			case render::ResolutionPreset::R_1440p: return { 2560, 1440 };
			case render::ResolutionPreset::R_4K:	return { 3840, 2160 };
			default: return { 1920, 1080 };
		}
	}

	glm::vec2 simManager::getInternalResolutionSizePx() const {
		return getPresetResolutionPx(); // no scale
	}

	void simManager::resetHDRToPreset() {
		_hdrUserOverride = false;
		const std::string hdr = getDefaultHDR();
		if (hdr != _activeHDRPath) { loadNewHDR(hdr); }
	}

	void simManager::reloadAllShaders() {
		_impl->_shaderBasic->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_basic.frag.glsl").string());
		_impl->_shaderLit->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_lit.frag.glsl").string());
		_impl->_shaderPBR->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_pbr.frag.glsl").string());

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

		if (scene::Input::IsKeyPressed(window, GLFW_KEY_W)) { processMovementKey(GLFW_KEY_W, kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_S)) { processMovementKey(GLFW_KEY_S, kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_A)) { processMovementKey(GLFW_KEY_A, kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_D)) { processMovementKey(GLFW_KEY_D, kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_SPACE)) { processMovementKey(GLFW_KEY_SPACE, kspd); }
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_LEFT_SHIFT)) { processMovementKey(GLFW_KEY_LEFT_SHIFT, kspd); }
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
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse);
	}

	// End Control Panel Helper
	void simManager::endSimManager() {
		ImGui::EndChild();
		ImGui::PopStyleVar(4);
	}
}