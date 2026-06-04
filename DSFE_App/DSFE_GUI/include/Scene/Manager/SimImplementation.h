// DSFE_GUI SimImplementation.h
#include "Scene/SimulationManager.h"
#include "Scene/Object.h"

#include <random>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <regex>

#include "Scene/Input.h"
#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Assets/MeshLoader.h"
#include "Scene/Light.h"
#include "Scene/AxisOrientator.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotModel.h"
#include "Robots/TrajectoryManager.h"

#include "Rendering/SkyboxRenderer.h"
#include "Rendering/ShaderUtil.h"
#include "Rendering/OpenGLBufferManager.h"
#include "Rendering/IBL.h"
#include "Rendering/Texture.h"

#include "Robots/RobotPresentationBuilder.h"
#include "Robots/RobotRenderer.h"

#include <filesystem>
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
		std::unordered_map<std::string, std::vector<scene::Object*>> _linkToObjects;
		std::unordered_map<std::string, scene::Object*> _primaryLinkObject;

		// Robot System
		std::unique_ptr<robots::RobotSystem> _robotSystem;	  // simulation

		// Robot Rendering
		std::unique_ptr <RobotRenderer> _robotRenderer; // rendering
		RobotRenderBinding _currentBinding; // current render binding 

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
			makeView(ViewID::Manual, { 0.0f, 0.5f, 1.0f }, 60.0f, target, { 0.0f, 1.0f, 0.0f });  // Default
			makeView(ViewID::Follow, { 3.0f, 0.25f, 0.0f }, 20.0f, target, { 0.0f, 1.0f, 0.0f }); // Follow
			// Ortho-ish
			makeView(ViewID::Top, { 0.0f, 2.0f, 0.0f }, 20.0f, target, { 0.0f, 0.0f, -1.0f }); // Top
			makeView(ViewID::Right, { 3.0f, 0.25f, 0.0f }, 20.0f, target, { 0.0f, 1.0f, 0.0f }); // Right
			makeView(ViewID::Front, { 0.0f, 0.25f, 3.0f }, 20.0f, target, { 0.0f, -1.0f, 0.0f }); // Front

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

			// Robot system with mesh loading (for normal simulation)
			_robotSystem = std::make_unique<robots::RobotSystem>();

			_robotRenderer = std::make_unique<RobotRenderer>();

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

		void clearRobotPresentation() {
			_primaryLinkObject.clear();
			_linkToObjects.clear();
			_objects.clear();
		}

		scene::Object* primaryObjectForLink(const std::string& linkName) {
			auto it = _primaryLinkObject.find(linkName);
			if (it == _primaryLinkObject.end()) { return nullptr; }
			return it->second;
		}

		void buildRobotPresentationFromModel(const robots::RobotModel& model, SimManager& owner) {
			clearRobotPresentation();

			RobotPresentationBuilder builder;
			RobotRenderBinding binding = builder.build(model);

			for (auto& owned : binding.ownedObjects) {
				_objects.push_back(std::move(owned));
			}

			_robotRenderer->bind(binding);

			for (const auto& [linkName, visuals] : binding.linkVisuals) {
				auto& target = _linkToObjects[linkName];

				for (auto* obj : visuals) {
					if (!obj) { continue; }

					target.push_back(obj);

					if (!_primaryLinkObject.contains(linkName)) {
						_primaryLinkObject[linkName] = obj;
					}
				}
			}
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
				return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
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
}