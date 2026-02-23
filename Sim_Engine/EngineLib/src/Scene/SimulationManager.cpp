#include "pch.h"
// File:   SimulationManager.cpp
// GitHub: SaltyJoss
#include "Scene/Object.h"
#include "Scene/SimulationManager.h"
#include "Scene/SimulationCore.h"

extern "C" core::ISimulationCore* CreateSimulationCore_v1();
extern "C" void DestroySimulationCore(core::ISimulationCore*);

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <random>

#include "Scene/Input.h"
#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Assets/MeshLoader.h"
#include "Scene/Light.h"
#include "Scene/AxisOrientator.h"

#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"

#include "Interpreter/IStoredProgram.h"
#include "Interpreter/StoredProgram.h"
#include "Interpreter/Parser.h"

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
	struct SimManager::Impl {
		// Completed Simulation Runs (thread-safe)
		std::mutex _completedRunsMutex;
		std::vector<StudyResult> _completedRuns;

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

		// Viewports
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
		shaders::Shader* currentShader;

		// Fullscreen Quad VAO
		GLuint _fullscreenVAO = 0;
		GLuint _worldGridVAO = 0;

		// Shadow Mapping (Cascaded)
		GLuint _cascadeFBO[SimManager::NUM_CASCADES]{};
		GLuint _cascadeDepth[SimManager::NUM_CASCADES]{};
		glm::mat4 _lightSpaceMatrixCascade[SimManager::NUM_CASCADES] = {};

		// Scene Objects
		std::unique_ptr<scene::Light> _light;
		std::unique_ptr<AxisOrientator> _axisOrientator;

		scene::Object* _selectedObject = nullptr;
		scene::Object* _cameraFollowTarget = nullptr;

		std::shared_ptr<scene::Mesh> _mesh;
		std::vector<std::unique_ptr<scene::Object>> _objects;

		// Physics System
		std::unique_ptr<physics::PhysicsSystem> _physics;
		// Robot System
		std::unique_ptr<robots::RobotSystem> _robotSystem;	  // simulation

		// Robot Follow Target
		bool eeFollowBound = false;
		scene::Object* eeObject = nullptr;

		// Trajectory Manager
		control::TrajectoryManager _traj;

		// SSAO Resources
		std::unique_ptr<shaders::Shader> _ssaoShader;
		std::unique_ptr<shaders::Shader> _ssaoBlurShader;
		GLuint _ssaoFBO = 0, _ssaoTex = 0;
		GLuint _ssaoBlurFBO = 0, _ssaoBlurTex = 0;
		GLuint _ssaoNoiseTex = 0;
		int _ssaoW = 0, _ssaoH = 0;
		std::vector<glm::vec3> _ssaoKernel;

		Impl(SimManager& owner) {
			_postShader = std::make_unique<shaders::Shader>();
			_postShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "post.frag.glsl").string());

			glGenVertexArrays(1, &_fullscreenVAO);

			// Lambda to create views
			auto makeView = [&](ViewID id, glm::vec3 pos, float fovDeg, glm::vec3 target, glm::vec3 /*upHint*/) {
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

			{
				// Top
				auto* camTop = _views[(size_t)ViewID::Top].cam.get();
				camTop->setFocus(target);
				camTop->setYaw(-glm::half_pi<float>());
				camTop->setPitch(-glm::half_pi<float>() + 0.001f);
				camTop->setOrbitDistance(2.0f);
				camTop->setMinDistance(0.5f);
				camTop->updateViewMatrix();

				// Right
				auto* camRight = _views[(size_t)ViewID::Right].cam.get();
				camRight->setFocus(target);
				camRight->setYaw(glm::pi<float>());
				camRight->setPitch(0.0f);
				camRight->setOrbitDistance(3.0f);
				camRight->setMinDistance(0.5f);
				camRight->updateViewMatrix();

				// Front
				auto* camFront = _views[(size_t)ViewID::Front].cam.get();
				camFront->setFocus(target);
				camFront->setYaw(-glm::half_pi<float>());
				camFront->setPitch(0.0f);
				camFront->setOrbitDistance(3.0f);
				camFront->setMinDistance(0.5f);
				camFront->updateViewMatrix();

				// Follow
				auto* camFollow = _views[(size_t)ViewID::Follow].cam.get();
				camFollow->setFocus(target);
				camFollow->setYaw(glm::pi<float>());
				camFollow->setPitch(0.0f);
				camFollow->setOrbitDistance(3.0f);
				camFollow->setMinDistance(0.5f);
				camFollow->updateViewMatrix();
			}

			// Shader Types A
			// Basic shader with no lighting
			_shaderBasic = std::make_shared<shaders::Shader>();
			_shaderBasic->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_basic.frag.glsl").string());
			// Lit shader with simple Blinn-Phong lighting
			_shaderLit = std::make_shared<shaders::Shader>();
			_shaderLit->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_lit.frag.glsl").string());
			// PBR shader with full Physically Based Rendering (for release visuals)
			_shaderPBR = std::make_shared<shaders::Shader>();
			_shaderPBR->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_pbr.frag.glsl").string());

			currentShader = _shaderPBR.get();
			_skybox = std::make_unique<render::SkyboxRenderer>();

			// Shader Types B
			_worldGridShader = std::make_unique<shaders::Shader>();
			_worldGridShader->load((paths::assets() / "shaders" / "world_grid.vert.glsl").string(), (paths::assets() / "shaders" / "world_grid.frag.glsl").string());

			_shadowShader = std::make_unique<shaders::Shader>();
			_shadowShader->load((paths::assets() / "shaders" / "shadow_depth.vert.glsl").string(), (paths::assets() / "shaders" / "shadow_depth.frag.glsl").string());

			// Light
			_light = std::make_unique<scene::Light>();
			_light->_isDirectional = true;

			// Axis Orientator
			_axisOrientator = std::make_unique<gui::AxisOrientator>();

			// World Grid VAO
			glGenVertexArrays(1, &_worldGridVAO);
			
			// Test Mesh
			_mesh = std::make_shared<scene::Mesh>();
			_mesh->init();

			// Physics system
			_physics = std::make_unique<physics::PhysicsSystem>();
			// Robot system with mesh loading (for normal simulation)
			_robotSystem = std::make_unique<robots::RobotSystem>(_objects, [&owner](const std::string& path) { return owner.loadMeshReturn(path); });

			// SSAO shaders
			_ssaoShader = std::make_unique<shaders::Shader>();
			_ssaoShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "ssao.frag.glsl").string());

			_ssaoBlurShader = std::make_unique<shaders::Shader>();
			_ssaoBlurShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "ssao_blur.frag.glsl").string());

			// Generate hemisphere kernel
			initSSAOKernel(64);

			// Generate noise texture (4x4 random rotation vectors)
			initSSAONoise();
		}

		// Random number generation for SSAO kernel and noise
		std::mt19937 _rng{ std::random_device{}() };
		std::uniform_real_distribution<float> _uni{ -1.0f, 1.0f };

		// Helper to get random float in [a,b]
		inline float rngFloat(float a, float b) {
			std::uniform_real_distribution<float> d(a, b);
			return d(_rng);
		}

		// Initialises the SSAO kernel with random samples in a hemisphere oriented along the positive Z axis, scaled to favor samples closer to the origin
		void initSSAOKernel(int size) {
			_ssaoKernel.clear();
			_ssaoKernel.reserve(size);
			for (int i = 0; i < size; ++i) {
				glm::vec3 sample(
					rngFloat(-1.0f, 1.0f),
					rngFloat(-1.0f, 1.0f),
					rngFloat(0.0f, 1.0f) // hemisphere: z in [0,1]
				);
				sample = glm::normalize(sample);
				sample *= rngFloat(0.0f, 1.0f);
				float scale = (float)i / (float)size;
				scale = 0.1f + scale * scale * 0.9f;
				sample *= scale;
				_ssaoKernel.push_back(sample);
			}
		}

		// Initializes the SSAO noise texture with random rotation vectors in the XY plane
		void initSSAONoise() {
			std::vector<glm::vec3> noise(16);
			// Generate 16 random rotation vectors in the XY plane (Z=0)
			for (int i = 0; i < 16; ++i) {
				noise[i] = glm::vec3(
					rngFloat(-1.0f, 1.0f),
					rngFloat(-1.0f, 1.0f),
					0.0f
				);
			}
			// Create OpenGL texture
			glGenTextures(1, &_ssaoNoiseTex);
			glBindTexture(GL_TEXTURE_2D, _ssaoNoiseTex);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, 4, 4, 0, GL_RGB, GL_FLOAT, noise.data());
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		}

		// Ensures SSAO FBOs and textures are created and match the given size, recreating them if necessary
		void ensureSSAOBuffers(int w, int h) {
			if (_ssaoW == w && _ssaoH == h) return;
			_ssaoW = w; _ssaoH = h;
			// Delete old buffers if they exist
			if (_ssaoFBO) { glDeleteFramebuffers(1, &_ssaoFBO); glDeleteTextures(1, &_ssaoTex); }
			if (_ssaoBlurFBO) { glDeleteFramebuffers(1, &_ssaoBlurFBO); glDeleteTextures(1, &_ssaoBlurTex); }
			// Lambda to create a single-channel floating point FBO and texture
			auto makeSingleChannelFBO = [](GLuint& fbo, GLuint& tex, int w, int h) {
				glGenFramebuffers(1, &fbo);
				glGenTextures(1, &tex);
				glBindFramebuffer(GL_FRAMEBUFFER, fbo);
				glBindTexture(GL_TEXTURE_2D, tex);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, w, h, 0, GL_RED, GL_FLOAT, nullptr);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
				glBindFramebuffer(GL_FRAMEBUFFER, 0);
			};
			// Create SSAO and blur FBOs/textures
			makeSingleChannelFBO(_ssaoFBO, _ssaoTex, w, h);
			makeSingleChannelFBO(_ssaoBlurFBO, _ssaoBlurTex, w, h);
		}

		// Renders the SSAO pass and subsequent blur pass, writing results to SSAO FBOs. Should be called after rendering the scene to the view's framebuffer (to provide depth texture input).
		void renderSSAO(SimManager& owner, Viewport& v) {
			if (!owner._settingsCurrent.ssao) return;
			// Determine SSAO buffer size based on division factor
			int ssaoDiv = std::max(1, owner._settingsCurrent.ssaoResDiv);
			int ssaoW = std::max(1, v.w / ssaoDiv);
			int ssaoH = std::max(1, v.h / ssaoDiv);
			ensureSSAOBuffers(ssaoW, ssaoH);
			// Clamp kernel size to available samples
			int kernelSize = std::min((int)_ssaoKernel.size(), owner._settingsCurrent.ssaoSamples);
			// SSAO Pass
			glBindFramebuffer(GL_FRAMEBUFFER, _ssaoFBO);
			glViewport(0, 0, ssaoW, ssaoH);
			glClear(GL_COLOR_BUFFER_BIT);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_BLEND);

			_ssaoShader->use();

			// Depth texture from the resolved scene FBO
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, v.fb->getDepthTexture());
			_ssaoShader->setInt1(0, "gDepth");
			// Noise texture
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, _ssaoNoiseTex);
			_ssaoShader->setInt1(1, "gNoise");

			// Camera matrices and parameters
			_ssaoShader->setMat4(v.cam->getProjection(), "projection");
			_ssaoShader->setMat4(glm::inverse(v.cam->getProjection()), "invProjection");
			_ssaoShader->setVec2(glm::vec2((float)ssaoW / 4.0f, (float)ssaoH / 4.0f), "noiseScale");
			_ssaoShader->setInt1(kernelSize, "kernelSize");
			_ssaoShader->setFlt1(0.25f, "radius");
			_ssaoShader->setFlt1(0.035f, "bias");
			_ssaoShader->setFlt1(owner._settingsCurrent.ssaoStrength, "strength");

			// SSAO kernel samples
			for (int i = 0; i < kernelSize; ++i) { _ssaoShader->setVec3(_ssaoKernel[i], "samples[" + std::to_string(i) + "]"); }

			glBindVertexArray(_fullscreenVAO);
			glDrawArrays(GL_TRIANGLES, 0, 3);

			// Blur pass
			glBindFramebuffer(GL_FRAMEBUFFER, _ssaoBlurFBO);
			glViewport(0, 0, ssaoW, ssaoH);
			glClear(GL_COLOR_BUFFER_BIT);

			// No need for depth test or blending for a simple fullscreen blur
			_ssaoBlurShader->use();
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, _ssaoTex);
			_ssaoBlurShader->setInt1(0, "ssaoInput");
			glDrawArrays(GL_TRIANGLES, 0, 3);
			glBindVertexArray(0);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
		}

		// Renders the given viewport to its framebuffer, handling dynamic resizing of the framebuffer and camera aspect ratio based on the provided display size
		void renderView(SimManager& owner, Viewport& v, int displayW, int displayH) {
			displayW = std::max(1, displayW);
			displayH = std::max(1, displayH);

            // Internal render target size
			glm::ivec2 rt(std::max(1, (int)owner._internalSize.x), std::max(1, (int)owner._internalSize.y));

			// In quad mode, render at the cell's display resolution to avoid scaling artifacts
			if (owner._impl->viewMode == Impl::ViewMode::Quad) {
				rt.x = std::max(1, displayW);
				rt.y = std::max(1, displayH);
			}
			const int rtW = std::max(1, (int)rt.x);
			const int rtH = std::max(1, (int)rt.y);

			LOG_INFO_ONCE("Rendering Viewport: RT Size = %dx%d, Display Size = %dx%d", rtW, rtH, displayW, displayH);

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

			// Update Follow Target: use the explicitly bound target (e.g. end-effector from loadRobot),
			// only fall back to _selectedObject if no explicit target was set via setViewFollowTarget
			if (&v == &_views[(size_t)gui::ViewID::Follow]) {
				scene::Object* target = v.followEnabled ? v.followTarget : _selectedObject;

				if (target && target->getMesh()) {
					glm::mat4 M = target->transform.toMatrix() * target->getMesh()->localTransform;
					glm::vec3 worldPos = glm::vec3(M[3]);
					v.cam->setFollowTarget(worldPos, target->transform.rotQ);
				}
			}

			// Skybox (renders before all opaque geometry, doesn't write depth)
			if (owner.skyboxEnabled) {
				glDepthMask(GL_FALSE);
				glDepthFunc(GL_LEQUAL);
				owner.SkyboxRender(v.cam.get());
				glDepthMask(GL_TRUE);
				glDepthFunc(GL_LESS);
			}

			// Meshes & World Grid
			GLint sampleBuffers = 0, samples = 0;
			glGetIntegerv(GL_SAMPLE_BUFFERS, &sampleBuffers);
			glGetIntegerv(GL_SAMPLES, &samples);
			LOG_INFO_ONCE("FB MSAA state: GL_SAMPLE_BUFFERS=%d GL_SAMPLES=%d", sampleBuffers, samples);

			owner.MeshRender(v.cam.get());
			if (owner._settingsCurrent.grid) { owner.WorldGridRender(v.cam.get(), v.w); }

			v.fb->unbind();

			// SSAO pass (reads resolved depth, writes to _ssaoBlurTex)
			renderSSAO(owner, v);

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
			_postShader->setInt1(1, "ssaoTex");
			_postShader->setBool(owner._settingsCurrent.ssao, "ssaoEnabled");
			_postShader->setFlt1(owner._settingsCurrent.exposure, "exposure");
			_postShader->setFlt1(owner._settingsCurrent.whitePoint, "whitePoint");
			_postShader->setVec2(glm::vec2(v.w, v.h), "uRes");

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, v.fb->getTexture());

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, owner._settingsCurrent.ssao ? _ssaoBlurTex : 0);

			glBindVertexArray(_fullscreenVAO);
			glDrawArrays(GL_TRIANGLES, 0, 3);
			if (owner._settingsCurrent.axisOrientator) { _axisOrientator->render(v.cam->getViewMatrix()); }
			glBindVertexArray(0);

			v.post->unbind();
		}

		static bool icontains(const std::string& s, const char* sub) {
			if (sub == nullptr || *sub == '\0') { return false; }

			// Case-insensitive search using std::search with a custom comparator
			auto it = std::search(
				s.begin(), s.end(),
				sub, sub + std::strlen(sub),
				[](char a, char b) {
					return std::tolower((unsigned char)a)
						== std::tolower((unsigned char)b);
				}
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

	// Helper: create CorePtr (unique_ptr with std::function deleter)
	static CorePtr makeCoreFactory() {
		core::ISimulationCore* raw = CreateSimulationCore_v1();
		if (!raw) {
			return CorePtr(nullptr, [](core::ISimulationCore*) {});
		}
		// std::function deleter is constructed from the lambda implicitly
		return CorePtr(raw, [](core::ISimulationCore* p) { DestroySimulationCore(p); });
	}

	// --------------------------------------------------
	//				CONSTRUCTOR & DESTRUCTOR
	// --------------------------------------------------

	SimManager::SimManager() : _internalSize(1920, 1080), _displaySize(1.0f, 1.0f), _backgroundColour(0.18f, 0.18f, 0.20f),
		_backgroundAlpha(1.0f), _impl(std::make_unique<Impl>(*this)), _core(std::make_unique<core::SimulationCore>()),
		_studyRunner(std::make_unique<StudyRunner>(makeCoreFactory, std::thread::hardware_concurrency() > 1 ? std::thread::hardware_concurrency() - 1 : 1)) {
		_core->setPhysicsSystem(_impl->_physics.get());
		_core->setRobotSystem(_impl->_robotSystem.get());
		_core->setObjects(&_impl->_objects);
		_core->setTrajectoryManager(&_impl->_traj);
	}

	// Initialises OpenGL resources, including framebuffers, shaders, and IBL. Also picks an internal resolution preset based on the display size to balance quality and performance.
	void SimManager::initGL() {
		if (_glReady) return;
		_glReady = true;

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

	// Helper to get the next ObjectID
	static inline scene::ObjectID next(scene::ObjectID id) { return static_cast<scene::ObjectID>(static_cast<std::uint32_t>(id) + 1); }

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


	// --------------------------------------------------
	//				    LIGHT & SKYBOX
	// --------------------------------------------------
	scene::Light* SimManager::getLight() { return _impl->_light.get(); }

	void SimManager::loadNewHDR(const std::string& path) {
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

	void SimManager::loadNewHDR_UI(const std::string& path) {
		loadNewHDR(path);
		_hdrUserOverride = true;
	}

	void SimManager::loadNewHDR_Preset(const std::string& path) {
		loadNewHDR(path);
		_hdrUserOverride = false;
	}

	// --------------------------------------------------
	//				CONTROL MODES & CAMERA
	// --------------------------------------------------

	// Get the active view camera
	scene::Camera* SimManager::getCamera() { return _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get(); }
	// Reset the active view camera to default position
	void SimManager::resetView() {
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

	// Attach camera to an object and start following it. The camera will maintain a fixed offset from the object's position and orientation.
	void SimManager::attachCameraToObject(scene::Object* obj) {
		if (!obj) return;
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();

		_impl->_cameraFollowTarget = obj;

		glm::vec3 pos = obj->transform.position;
		glm::quat rot = obj->transform.rotQ;

		cam->startFollow(pos, rot, glm::vec3(0, 2, 5));
	}

	// Detach camera from any object and stop following
	void SimManager::detachCameraFromObject() {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		_impl->_cameraFollowTarget = nullptr;
		cam->clearFollow();
	}

	// Set the follow target for a specific view
	void SimManager::setViewFollowTarget(ViewID view, scene::Object* obj, const glm::vec3& offset) {
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
	void SimManager::clearViewFollowTarget(ViewID view) {
		if (view < ViewID::Manual || view >= ViewID::COUNT) { return; }
		auto& v = _impl->_views[static_cast<size_t>(view)];
		v.followTarget = nullptr;
		v.followEnabled = false;
		if (v.cam) { v.cam->clearFollow(); }
	}

	// Convenience for Follow view: set the follow target to the object attached to a robot joint (e.g. end-effector)
	bool SimManager::setViewFollowRobotJoint(ViewID view, const std::string& jointName, const glm::vec3& offset) {
		if (!hasRobot()) {
			LOG_WARN("setViewFollowRobotJoint: no robot loaded");
			return false;
		}

		robots::RobotSystem* rs = robotSystem();
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

	// Convenience for Follow view
	bool SimManager::followRobotJoint(const std::string& jointName, const glm::vec3& offset) {
		return setViewFollowRobotJoint(gui::ViewID::Follow, jointName, offset);
	}

	// --------------------------------------------------
	//			    MESH LOADING & GEOMETRY
	// --------------------------------------------------

	// Load a mesh from file and create one Object per submesh. The last loaded mesh becomes the active selection.
	void SimManager::loadMesh(const std::string& filepath) {
		assets::MeshLoader loader;
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

	// Returns the loaded objects so they can be used as targets for robot joints in the same frame (e.g. end-effector)
	std::vector<scene::Object*> SimManager::loadMeshReturn(const std::string& filepath) {
		assets::MeshLoader loader;
		auto meshes = loader.load(filepath);
		std::vector<scene::Object*> result;
		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			auto raw = obj.get();
			raw->internal = true;
			_impl->_objects.push_back(std::move(obj));
			result.push_back(raw);
		}
		return result;
	}

	// Setter and Getter for the active mesh
	void SimManager::setMesh(std::shared_ptr<scene::Mesh> mesh) { _impl->_mesh = mesh; }
	std::shared_ptr<scene::Mesh> SimManager::getMesh() { return _impl->_mesh; }

	// Set the currently selected object (can be nullptr to deselect)
	void SimManager::setSelectedObject(scene::Object* obj) { _impl->_selectedObject = obj; }
	// Add a new object to the scene and select it
	void SimManager::addObject(std::unique_ptr<scene::Object> obj) { _impl->_objects.push_back(std::move(obj)); } // Cache the unique_ptr

	// Remove an object by index
	void SimManager::deleteObject(int index) {
		if (index < 0 || index >= _impl->_objects.size()) { return; }
		if (_impl->_selectedObject == _impl->_objects[index].get()) { _impl->_selectedObject = nullptr; }
		_impl->_objects.erase(_impl->_objects.begin() + index);
	}

	// Remove an object by pointer
	void SimManager::removeObject(scene::Object* obj) {
		if (!obj) return;

		auto it = std::remove_if(
			_impl->_objects.begin(),
			_impl->_objects.end(),
			[obj](const std::unique_ptr<scene::Object>& o) {
				return o.get() == obj;
			}
		);

		_impl->_objects.erase(it, _impl->_objects.end());
	}

	// Access the objects as raw pointers for use in the rest of the codebase, while maintaining ownership in SimManager
	std::vector<std::unique_ptr<scene::Object>>& SimManager::getObjects() { return _impl->_objects; }
	// Get the currently selected object (can be nullptr)
	scene::Object* SimManager::getObject() { return _impl->_selectedObject; }

	// Helper to find an object by its ID (returns nullptr if not found)
	scene::Object* SimManager::getObjectByID(scene::ObjectID id) {
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

	// Main render function called by the application
	void SimManager::render() {
		if (hasCompletedStudy()) {
			auto results = consumeCompletedStudy();

			for (const auto& r : results) {
				LOG_INFO("Study completed: %s", r.tag.c_str());
				// TODO: update plots, telemetry graphs, UI panels here
			}
		}

		ImGuiIO& io = ImGui::GetIO();
		_core->tick(io.DeltaTime);
		_fpsCounter.update();

		drawMainDockspace();
		drawViewportWindow();
	}

	// --- UI Elements ---

	// Main Dockspace with Menu Bar
	void SimManager::drawMainDockspace() {
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
	void SimManager::drawViewportWindow() {
		ImGui::Begin("Viewport", nullptr,
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse);

		beginSimManager("##ViewportBody");

		// --- Tabs: Single / Quad ---
		if (ImGui::BeginTabBar("ViewportTabs", ImGuiTabBarFlags_None)) {
			const bool singleSelected = ImGui::BeginTabItem("Single");
			if (singleSelected) {
				// Restore to Manual view when switching back from Quad
				if (_impl->viewMode == Impl::ViewMode::Quad) {
					_impl->activeView = gui::ViewID::Manual;
				}
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

			// Helper to draw each cell with the same pattern
			auto drawCell = [&](const char* childId, gui::ViewID id, bool sameLine) {
				if (sameLine) ImGui::SameLine();
				ImGui::BeginChild(childId, cell, false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
				
				auto& v = _impl->_views[(size_t)id];
				ImVec2 inner = ImGui::GetContentRegionAvail();

				ImGui::Image((ImTextureID)(intptr_t)v.post->getTexture(), inner, ImVec2(0, 1), ImVec2(1, 0));

				// Set active view when clicking inside the quad cell
				if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
					_impl->activeView = id;
				}

				// Handle scroll zoom on hovered quad cell
				if (ImGui::IsWindowHovered()) {
					float scrollY = ImGui::GetIO().MouseWheel;
					if (scrollY != 0.0f) {
						v.cam->onMouseWheel((double)scrollY);
					}
				}

				// Visual indication: draw a border around the active cell
				if (_impl->activeView == id) {
					ImDrawList* dl = ImGui::GetWindowDrawList();
					ImVec2 p0 = ImGui::GetWindowPos();
					ImVec2 p1 = ImVec2(p0.x + ImGui::GetWindowSize().x, p0.y + ImGui::GetWindowSize().y);
					dl->AddRect(p0, p1, IM_COL32(255, 200, 0, 180), 4.0f, 0, 2.0f);
				}

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

	void SimManager::resize(int32_t width, int32_t height) {
		if (width <= 0 || height <= 0) return;
		_internalSize = { (float)width, (float)height };

		// Force per-view reallocation next frame
		for (auto& v : _impl->_views) {
			v.w = 0; v.h = 0;
			v.displayW = 0; v.displayH = 0;
		}

		LOG_INFO("Resized SimManager INTERNAL RT to %dx%d", width, height);
	}

	// Accessor to core's updatePhysics for use in the main application loop
	void SimManager::updatePhysics(double dt) { _core->updatePhysics(dt); }
	
	// Load a robot by name from the robot system
	void SimManager::loadRobot(const std::string& name) {
		// Clear any existing robot first
		if (hasRobot()) { clearRobot(); }

		setSelectedObject(nullptr);
		detachCameraFromObject();
		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;

		if (!_impl->_robotSystem) return;

		const size_t startIdx = _impl->_objects.size();
		_impl->_robotSystem->loadRobot(name);

		// Reset adaptive state for reference system
		if (auto* simInteg = _impl->_robotSystem->getIntegrator()) { 
			simInteg->resetAdaptiveState();
		}

		// Attempt to find an end-effector candidate among the newly added objects and bind the Follow view to it
		scene::Object* ee = _impl->findEndEffectorFromRange(startIdx);
		if (ee) {
			_impl->eeObject = ee;
			setViewFollowTarget(gui::ViewID::Follow, ee, glm::vec3(0.0f, 0.2f, 0.6f));
			_impl->eeFollowBound = true;
			LOG_INFO("Follow view bound to end-effector candidate: %s", ee->name.c_str());
		}

		// Auto-select the first object of the newly loaded robot
		if (startIdx < _impl->_objects.size()) {
			setSelectedObject(_impl->_objects[startIdx].get());
		}

		//_impl->_robotSystem->setDefaultPoseDeg();
	}
	void SimManager::setRobotLinkRotation(const std::string& linkName, double angle) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotLinkRotation(linkName, angle); }
	}
	void SimManager::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootPose(pos, rot); }
	}
	void SimManager::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootHome(pos, rot); }
	}
	void SimManager::resetRobot() { 
		if (_impl->_robotSystem) { _impl->_robotSystem->resetRobot(); }
	}
	void SimManager::clearRobot() {
		setSelectedObject(nullptr); // deselect any selected object
		if (_impl->_robotSystem) { _impl->_robotSystem->clearRobot(); }

		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;
	}
	const bool SimManager::hasRobot() const { return _impl->_robotSystem && _impl->_robotSystem->hasRobot(); }

	// Access the physics system (non-const and const versions)
	physics::PhysicsSystem* SimManager::physicsSystem() { return _core->physicsSystem(); }
	const physics::PhysicsSystem* SimManager::physicsSystem() const { return _core->physicsSystem(); }

	// Access the robot system (non-const and const versions)
	robots::RobotSystem* SimManager::robotSystem() { return _core->robotSystem(); }
	const robots::RobotSystem* SimManager::robotSystem() const { return _core->robotSystem(); }

	// Access the trajectory manager (non-const and const versions)
	control::TrajectoryManager* SimManager::traj() { return _core->trajectoryManager(); }
	const control::TrajectoryManager* SimManager::traj() const { return _core->trajectoryManager(); }

	// Start the simulation
	void SimManager::startSimulation() {
		if (!hasRobot()) {
			LOG_WARN("Cannot start simulation: no robot loaded");
			return;
		}
		_core->startSimulation();
	}
	// Stop the simulation
	void SimManager::stopSimulation() { _core->stopSimulation(); }

	// Check if the simulation is currently running
	const bool SimManager::isSimRunning() const { return _core->isSimRunning(); }

	// Setter for current simulation time (in seconds)
	void SimManager::setSimTime(double time) { _core->setSimTime(time); }
	const double SimManager::simTime() const { return _core->simTime(); }

	// Setter and gettter for fixed timstep (in seconds)
	void SimManager::setFixedDt(double dt) { _core->setFixedDt(dt); }
	const double SimManager::fixedDt() const { return _core->fixedDt(); }

	// Setter and getter for telemetry frequency (in Hz)
	void SimManager::setTelemetryHz(double hz) { _core->setTelemetryHz(hz); }
	const double SimManager::telemetryHz() const { return _core->telemetryHz(); }

	// Set whether a script is currently running (used to disable UI elements, etc.)
	void SimManager::setScriptRunning(bool running) { _core->setScriptRunning(running); }
	const bool SimManager::isScriptRunning() const { return _core->isScriptRunning(); }

	// Setters and getters for last script text
	void SimManager::setLastScriptText(const std::string& text) { _core->setLastScriptText(text); }
	const std::string& SimManager::lastScriptText() const { return _core->lastScriptText(); }

	// Accessors for the Simulation Core's telemetry data
	diagnostics::TelemetryRecorder& SimManager::telemetry() { return _core->telemetry(); }
	const diagnostics::TelemetryRecorder& SimManager::telemetry() const { return _core->telemetry(); }

	// Accesors for the active program (if any)
	void SimManager::setActiveProgram(interpreter::IStoredProgram* program) { _core->setActiveProgram(program); }
	interpreter::IStoredProgram* SimManager::activeProgram() { return _core->activeProgram(); }
	const interpreter::IStoredProgram* SimManager::activeProgram() const { return _core->activeProgram(); }

	// Access the simulation core interface (non-const and const versions)
	core::ISimulationCore* SimManager::simCoreInterface() { return _core.get(); }
	const core::ISimulationCore* SimManager::simCoreInterface() const { return _core.get(); }

	// Access the concrete simulation core (non-const and const versions)
	core::SimulationCore* SimManager::simCore() { return _core.get(); }
	const core::SimulationCore* SimManager::simCore() const { return _core.get(); }

	// Helper to replace the integrator method in the script text
	// A hacky approach my idea, but it works for me and honestly im starting to write up the dissertation so IT WILL DO :)
	// PS:If anyone has any better solution msg me

	// This seems to be the better solution?
	static std::string replaceIntegratorInScript(const std::string& script, const std::string& methodName) {
		std::regex re(R"((?i)set\s*\(\s*integrator\s*,\s*([a-z0-9_]+)\s*\))"); // case-insensitive regex to match my DSL command -> set(integrator, method)
		std::string replacement = "set(integrator, " + methodName + ")";
		return std::regex_replace(script, re, replacement);
	}

	// Run a script to completion synchronously with a specific integrator
	bool SimManager::runScriptToCompletion(const std::string& scriptText, integration::eIntegrationMethod method) {
		if (!hasRobot()) { return false; }

		// Map method enum to string name
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45" };
		const std::string methodName = names[static_cast<int>(method)];

		// Replace the integrator method in the script text
		std::string modifiedScript = replaceIntegratorInScript(scriptText, methodName);

		// Create program and parser (bound to headless core)
		auto program = std::make_unique<interpreter::StoredProgram>(_core.get());
		if (scene::Object* o = getObject()) program->setDefaultObject(o);
		auto parser = std::make_unique<interpreter::Parser>(program.get());

		// Parse the modified script and start the program
		parser->parse(modifiedScript);
		program->start();
		return _core->runScriptToCompletion(program.get(), method); // this will block until the script finishes
	}

	// --------------------------------------------------
	//			 INTERNAL REDNDERING PIPELINE
	// --------------------------------------------------

	// Initialize shadow map resources for cascaded shadow mapping
	void SimManager::InitShadowResource(int baseRes) {
		if (_shadowsInit) {
			glDeleteFramebuffers(SimManager::NUM_CASCADES, _impl->_cascadeFBO);
			glDeleteTextures(SimManager::NUM_CASCADES, _impl->_cascadeDepth);
		}

		glGenFramebuffers(SimManager::NUM_CASCADES, _impl->_cascadeFBO);
		glGenTextures(SimManager::NUM_CASCADES, _impl->_cascadeDepth);

		for (int i = 0; i < SimManager::NUM_CASCADES; i++) {
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

	// Initialize the IBL system with a default HDR environment map
	void SimManager::InitIBL() {
		_impl->_ibl = std::make_unique<render::IBL>();
		_impl->_ibl->init((paths::assets() / "hdr" / "default_white.hdr").string());
	}

	// Render the world grid overlay in the viewport
	void SimManager::WorldGridRender(scene::Camera* cam, int rtW) {
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

		// Scale grid line thickness so smaller RTs (quad mode) match single view appearance
		const float fullW = std::max(1.0f, _internalSize.x);
		const float internalScale = (float)std::max(1, rtW) / fullW;

		_impl->_worldGridShader->use();
		_impl->_worldGridShader->setMat4(cam->getViewProjection(), "gVP");
		_impl->_worldGridShader->setMat4(cam->getViewMatrix(), "gView");
		_impl->_worldGridShader->setVec3(cam->getPosition(), "gCameraWorldPos");
		_impl->_worldGridShader->setFlt1(_settingsCurrent.renderScale, "gRenderScale");
		_impl->_worldGridShader->setFlt1(internalScale, "gInternalScale");

		glBindVertexArray(_impl->_worldGridVAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glBindVertexArray(0);

		glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);

		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glDepthFunc(GL_LESS);
	}

	// Render the meshes in the scene using the currently selected shader mode, setting appropriate uniforms for each mode
	void SimManager::MeshRender(scene::Camera* cam) {
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

		// Main mesh rendering loop
		for (auto& obj : _impl->_objects) {
			if (!obj || !obj->getMesh()) continue;

			if (_impl->_cameraFollowTarget == obj.get()) { cam->setFollowTarget(obj->transform.position, obj->transform.rotQ); }

			glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;
			shader->setMat4(model, "model");

			// Per-mode material uniforms
			switch (currentShaderMode) {
				case ShaderMode::Basic:
					// (IMPORTANT) mesh_basic.frag needs: uniform vec3 color;
					shader->setVec3(obj->getMesh()->getAlbedo(), "albedo");
					break;

				case ShaderMode::Lit:
					// (IMPORTANT) mesh_lit.frag needs: albedo, lightPosition, lightColour, lightIntensity, camPos
					shader->setVec3(obj->getMesh()->getAlbedo(), "albedo");
					shader->setVec3(_impl->_light->getPosition(), "lightPosition");
					shader->setFlt1(_impl->_light->getIntensity(), "lightIntensity");
					shader->setVec3(_impl->_light->getColour(), "lightColour");
					shader->setVec3(cam->getPosition(), "camPos");
					break;

				case ShaderMode::PBR:
					// Per-mesh PBR material properties
					shader->setVec3(obj->getMesh()->getAlbedo(), "albedo");
					shader->setFlt1(obj->getMesh()->getMetallic(), "metallic");
					shader->setFlt1(obj->getMesh()->getRoughness(), "roughness");
					shader->setFlt1(1.0f, "ao");
					shader->setFlt1(_settingsCurrent.ambientStrength, "ambientStrength");

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

	// Render the shadow maps for each cascade by rendering the scene from the light's perspective into the depth textures
	void SimManager::ShadowPass(scene::Camera* cam) {
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

	// Compute the light-space matrix for a given camera frustum slice (cascade)
	glm::mat4 SimManager::LightSpaceMatrix(scene::Camera* cam, float nearPlane, float farPlane) {
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

	// Render the skybox using the IBL environment cubemap
	void SimManager::SkyboxRender(scene::Camera* cam) {
		glm::mat4 view = cam->getViewMatrix();
		glm::mat4 projection = cam->getProjection();

		_impl->_skybox->setEnvironmentTexture(_impl->_ibl->getEnvCubemap());
		_impl->_skybox->render(projection, view);
	}

	// Load a new HDR environment map for IBL
	std::string SimManager::getDefaultHDR() const { return (paths::assets() / "hdr"/ "default_white.hdr").string(); }

	// Load a new HDR environment map for IBL
	const shaders::Shader* SimManager::getCurrentShader() const { return _impl->currentShader; }
	void SimManager::applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r) { applyRenderProfile(s, r); }

	void SimManager::applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r) {
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

	// Get the pixel dimensions for the current resolution preset
	glm::vec2 SimManager::getPresetResolutionPx() const {
		switch (_resCurrent) {
			case render::ResolutionPreset::R_720p:  return { 1280, 720 };
			case render::ResolutionPreset::R_1080p: return { 1920, 1080 };
			case render::ResolutionPreset::R_1440p: return { 2560, 1440 };
			case render::ResolutionPreset::R_4K:	return { 3840, 2160 };
			default: return { 1920, 1080 };
		}
	}

	glm::vec2 SimManager::getInternalResolutionSizePx() const {
		return getPresetResolutionPx(); // no scale
	}

	void SimManager::resetHDRToPreset() {
		_hdrUserOverride = false;
		const std::string hdr = getDefaultHDR();
		if (hdr != _activeHDRPath) { loadNewHDR(hdr); }
	}

	void SimManager::reloadAllShaders() {
		_impl->_shaderBasic->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_basic.frag.glsl").string());
		_impl->_shaderLit->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_lit.frag.glsl").string());
		_impl->_shaderPBR->load((paths::assets() / "shaders" / "vs_pbr.vert.glsl").string(), (paths::assets() / "shaders" / "mesh_pbr.frag.glsl").string());
		_impl->_ssaoShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "ssao.frag.glsl").string());
		_impl->_ssaoBlurShader->load((paths::assets() / "shaders" / "post.vert.glsl").string(), (paths::assets() / "shaders" / "ssao_blur.frag.glsl").string());

		LOG_INFO("All shaders reloaded from disk.");
		D_INFO_ONCE("All shaders reloaded from disk.");
	}

	void SimManager::setLightColour(const glm::vec3& colour) { _impl->_light->_colour = colour; }

	// --------------------------------------------------
	//					INPUT HANDLING
	// --------------------------------------------------
	void gui::SimManager::processMovementKey(int key, float delta) {
		if (_impl->viewMode == Impl::ViewMode::Quad) { return; }// No keyboard movement in quad view
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		if (ctrlMode == ControlMode::Camera) { cam->processKeyboard(key, delta); }
		else if (ctrlMode == ControlMode::Object && _impl->_mesh) { /*idea is to add multiple angles to switch between!*/ }
	}

	void gui::SimManager::handleContinuousMovement(GLFWwindow* window, float dt) {
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

	void gui::SimManager::handleMouseLook(GLFWwindow* window, double xpos, double ypos) {
		if (_impl->viewMode == Impl::ViewMode::Quad) { return; } // No mouse look in quad view
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) { return; }

		bool captured = true;
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

	void SimManager::onMouseMove(double x, double y, scene::eInputButton button) {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		glm::vec2 pos2d{ x, y };
		glm::vec2 delta = pos2d - _lastMousePos;
		_lastMousePos = pos2d;

		if (_impl->viewMode == Impl::ViewMode::Quad) { return; } // No mouse drag in quad view

		if (!_isHovered) {
			cam->setCurrentPos2D(pos2d);
			_impl->_selectedObject->setLastMousePos(pos2d);
			return;
		}

		if (ctrlMode == ControlMode::Camera) { cam->onMouseMove(x, y, button); }
		else if (ctrlMode == ControlMode::Object && _impl->_selectedObject) { _impl->_selectedObject->onMouseMove(x, y, button); }
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

	// --- Helpers ---

	// Begin Control Panel Helper
	void SimManager::beginSimManager(const char* id) {
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
	void SimManager::endSimManager() {
		ImGui::EndChild();
		ImGui::PopStyleVar(4);
	}
}