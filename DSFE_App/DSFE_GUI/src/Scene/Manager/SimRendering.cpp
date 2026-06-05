// DSFE_GUI SimRendering.cpp
#include "Scene/Object.h"
#include "Scene/SimulationManager.h"
#ifdef __gl_h_
#undef __gl_h_
#endif
#include "Manager/SimImplementation.h"

namespace gui {
	scene::Light* SimManager::getLight() { return _impl->_light.get(); }

	void SimManager::loadNewHDR(const std::string& path) {
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

	// Initialise the IBL system with a default HDR environment map
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
		_impl->_worldGridShader->setFlt1(2500.0f, "gGridSize");

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

			shader->setFltArray2(splitDist0, splitDist1, "cascadeSplits");
		}

		// Update camera and light uniforms (only those relevant to the current shader mode)
		cam->update(shader);
		_impl->_light->update(shader);

		// Main mesh rendering loop
		for (auto& obj : _impl->_objects) {
			if (!obj || !obj->getMesh()) { continue; }
			if (_impl->_cameraFollowTarget == obj.get()) { cam->setFollowTarget(obj->transform.position, obj->transform.rotQ); }
			glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;
			shader->setMat4(model, "model");

			// Per-mode material uniforms
			switch (currentShaderMode) {
				case ShaderMode::Basic:
				// (IMPORTANT) mesh_basic.frag needs: uniform vec3 color;
				shader->setVec3(obj->material.albedo, "albedo");
				break;

				case ShaderMode::Lit:
				// (IMPORTANT) mesh_lit.frag needs: albedo, lightPosition, lightColour, lightIntensity, camPos
				shader->setVec3(obj->material.albedo, "albedo");
				shader->setVec3(_impl->_light->getPosition(), "lightPosition");
				shader->setFlt1(_impl->_light->getIntensity(), "lightIntensity");
				shader->setVec3(_impl->_light->getColour(), "lightColour");
				shader->setVec3(cam->getPosition(), "camPos");
				break;

				case ShaderMode::PBR:
				// Per-mesh PBR material properties
				shader->setVec3(obj->material.albedo, "albedo");
				shader->setFlt1(obj->material.metallic, "metallic");
				shader->setFlt1(obj->material.roughness, "roughness");
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
		glBindFramebuffer(GL_FRAMEBUFFER, _presentationFBO);
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
	std::string SimManager::getDefaultHDR() const { return (paths::assets() / "hdr" / "default_white.hdr").string(); }

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

		//LOG_INFO("Render settings applied: resPreset=%d shadowRes=%d msaa=%d renderScale=%.2f", (int)r, _settingsCurrent.shadowMapRes, _settingsCurrent.msaaSamples, _settingsCurrent.renderScale);
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

		//LOG_INFO("All shaders reloaded from disk.");
		D_INFO_ONCE("All shaders reloaded from disk.");
	}

	void SimManager::setLightColour(const glm::vec3& colour) { _impl->_light->_colour = colour; }
}