
#include "pch.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include "Scene/SceneView.h"
#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/Light.h"
#include "Scene/Input.h"
#include "Scene/Object.h"

#include "Rendering/Cubemap.h"
#include "Rendering/Skybox.h"
#include "Rendering/ShaderUtil.h"
#include "Rendering/OpenGLBufferManager.h"
#include "Rendering/IBL.h"

#include <Platform/WindowManager.h>

#include "EngineLib/LogMacros.h"

namespace gui{

	SceneView::SceneView() :
		_camera(nullptr), _frameBuffer(nullptr), _shader(nullptr), _light(nullptr),
		_worldGridShader(nullptr), _shadowShader(nullptr), _size(1920, 1080)
	{
		_frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
		_frameBuffer->createBuffers(1920, 1080);

		_shader = std::make_unique<shaders::Shader>();
		_shader->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/fs_pbr.frag.glsl");

		std::array<std::string, 6> facesCubemap = {
			"Engine/assets/cubemaps/" + folder + "/px.png",
			"Engine/assets/cubemaps/" + folder + "/nx.png",
			"Engine/assets/cubemaps/" + folder + "/py.png",
			"Engine/assets/cubemaps/" + folder + "/ny.png",
			"Engine/assets/cubemaps/" + folder + "/pz.png",
			"Engine/assets/cubemaps/" + folder + "/nz.png"
		};

		_skyboxShader = std::make_unique<shaders::Shader>();
		_skyboxShader->load("Engine/assets/shaders/skybox.vert.glsl", "Engine/assets/shaders/skybox.frag.glsl");

		_cubemap = std::make_unique<render::Cubemap>(facesCubemap);
		_skybox = std::make_unique<render::Skybox>(_cubemap.get(), _skyboxShader.get());

		_worldGridShader = std::make_unique<shaders::Shader>();
		_worldGridShader->load("Engine/assets/shaders/world_grid.vert.glsl", "Engine/assets/shaders/world_grid.frag.glsl");

		_shadowShader = std::make_unique<shaders::Shader>();
		_shadowShader->load("Engine/assets/shaders/shadow_depth.vert.glsl", "Engine/assets/shaders/shadow_depth.frag.glsl");

		_light = std::make_unique<elements::Light>();
		_camera = std::make_unique<elements::Camera>(glm::vec3(0, 15, 20), 45.0f, 16 / 9, 0.5f, 2000.0f);

		glGenVertexArrays(1, &_worldGridVAO);

		_mesh = std::make_shared<elements::Mesh>();
		_mesh->init();

		_object = std::make_shared<elements::Object>(_mesh);

		if (_checkerPlane) _checkerPlane->clear();
		_checkerPlane = createCheckerPlane(50.0f);
		planeY = 2.5f;

		InitShadowResource();
		InitIBL();
	}

	SceneView::~SceneView()
	{
		_shader->unload();
		if (_frameBuffer) _frameBuffer->deleteBuffers();
		if (_mesh) _mesh->clear();
		if (_checkerPlane) _checkerPlane->clear();
	}

	void SceneView::render() {
		_fpsCounter.update();
		ShadowPass();

		_frameBuffer->bind();

		WorldGridRender();
		MeshRender();
		
		if (skyboxEnabled) { SkyboxRender(); }
		
		_frameBuffer->unbind();

		ImGui::Begin("Game Engine");

		ImGui::Text("FPS: %.1f", _fpsCounter.getFPS());

		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);
		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image((void*)textureID, viewportPanelSize, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}

	void SceneView::resize(int32_t width, int32_t height) {
		// update camera projection
		float aspect = (float)width / (float)height;
		_camera->setAspect(aspect);

		// store updated size
		_size = glm::vec2(width, height);

		// rebuild framebuffer
		_frameBuffer->deleteBuffers();
		_frameBuffer->createBuffers(width * 2.0f, height * 2.0f);

		LOG_INFO("Framebuffer resized: %d x %d", width, height);
	}

	std::shared_ptr<elements::Mesh> gui::SceneView::createCheckerPlane(float size) {

		auto plane = std::make_shared<elements::Mesh>();

		std::vector<glm::vec3> pos = {
			{-size, planeHeight, -size},
			{ size, planeHeight, -size},
			{ size, planeHeight,  size},
			{-size, planeHeight,  size}
		};

		glm::vec3 normal(0.0f, 1.0f, 0.0f);

		for (auto& p : pos) {
			elements::VertexHolder vh(p, normal);
			plane->addVertex(vh);
		}

		plane->addVertexIndex(0);
		plane->addVertexIndex(1);
		plane->addVertexIndex(2);
		plane->addVertexIndex(2);
		plane->addVertexIndex(3);
		plane->addVertexIndex(0);

		plane->init();
		return plane;
	}

	void SceneView::onMouseWheel(double delta) { 
		if (!_isHovered) return;

		if (_controlMode == ControlMode::Camera) _camera->onMouseWheel(delta);
		else if (_controlMode == ControlMode::Object && _mesh) _mesh->_position.z += (float)delta * 0.1f;
	}

	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) { 
		glm::vec2 pos2d{ x, y };
		glm::vec2 delta = pos2d - _lastMousePos;
		_lastMousePos = pos2d;

		if (!_isHovered) {
			_camera->setCurrentPos2D(pos2d);
			_object->setLastMousePos(pos2d);
			return;
		}

		if (_controlMode == ControlMode::Camera) {
			_camera->onMouseMove(x, y, button);
		}
		else if (_controlMode == ControlMode::Object && _object) {
			_object->onMouseMove(x, y, button);
		}
	}

	void SceneView::loadMesh(const std::string& filepath) {
		if (!_mesh) _mesh = std::make_shared<elements::Mesh>();
		else _mesh->clear(); // implement clear() to delete VAO/VBO etc.
		_mesh->load(filepath);

		_mesh->_position = glm::vec3(0.0f);

		LOG_INFO("Mesh loaded and centered from %s", filepath.c_str());
	}

	void SceneView::resetView() {
		_camera->reset();
	}

	/*
	 * --------------------------------------------
	 *				RENDERING METHODS
	 * --------------------------------------------
	 */

	void SceneView::WorldGridRender() {
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		glClearColor(_backgroundColour.r, _backgroundColour.g, _backgroundColour.b, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		_worldGridShader->use();
		_worldGridShader->setMat4(_camera->getViewProjection(), "gVP");
		_worldGridShader->setVec3(_camera->getPosition(), "gCameraWorldPos");

		glBindVertexArray(_worldGridVAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);

		glDepthMask(GL_TRUE);
	}

	void SceneView::MeshRender() {
		_shader->use();

		for (int i = 0; i < NUM_CASCADES; i++) {
			glActiveTexture(GL_TEXTURE5 + i);
			glBindTexture(GL_TEXTURE_2D, _cascadeDepth[i]);
			_shader->setInt1(5 + i, "cascadeShadowMap[" + std::to_string(i) + "]");
			_shader->setMat4(_lightSpaceMatrixCascade[i], "lightSpaceMatrix[" + std::to_string(i) + "]");
		}

		_shader->setFlt2(_cascadeSplits[0], _cascadeSplits[1], "cascadeSplits");

		_camera->update(_shader.get());
		_light->update(_shader.get());

		// Render checker floor
		if (_checkerPlane) {
			glm::mat4 floorModel(1.0f);
			_shader->setMat4(floorModel, "model");

			// Set checkerboard uniforms
			_shader->setBool(true, "isFloor");
			_shader->setVec3(glm::vec3(1.0f), "colour1");
			_shader->setVec3(glm::vec3(0.0f), "colour2");
			_shader->setFlt1(1.0f, "checkSize");

			_checkerPlane->update(_shader.get());
			_checkerPlane->render();
		}

		if (_object && _object->getMesh()) {
			glm::mat4 model(1.0f);

			_shader->setMat4(glm::translate(glm::mat4(1.0f), _object->getMesh()->_position), "model");
			_shader->setBool(false, "isFloor");            // mark as non-floor
			_object->getMesh()->update(_shader.get());
			_object->getMesh()->render();
		}
	}

	glm::mat4 SceneView::LightSpaceMatrix(float nearPlane, float farPlane) {
		std::array<glm::vec4, 8> corners = _camera->getFrustumCornersWorldSpace(nearPlane, farPlane);

		glm::vec3 lightDir = glm::normalize(_light->getDirection());

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

		glm::mat4 lightProj = glm::ortho(minX, maxX, minY, maxY, minZ - 20.0f, maxZ + 20.0f);

		return lightProj * lightView;
	}

	void SceneView::InitShadowResource() {
		int shadowRes[NUM_CASCADES] = { 4096, 2048 };

		glGenFramebuffers(NUM_CASCADES, _cascadeFBO);
		glGenTextures(NUM_CASCADES, _cascadeDepth);

		for (int i = 0; i < NUM_CASCADES; i++) {
			glBindTexture(GL_TEXTURE_2D, _cascadeDepth[i]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F,
				shadowRes[i], shadowRes[i], 0,
				GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			float border[] = { 1,1,1,1 };
			glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

			glBindFramebuffer(GL_FRAMEBUFFER, _cascadeFBO[i]);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
				GL_TEXTURE_2D, _cascadeDepth[i], 0);

			glDrawBuffer(GL_NONE);
			glReadBuffer(GL_NONE);
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void SceneView::InitIBL()
	{
		_ibl = std::make_unique<render::IBL>();
		_ibl->init("Engine/assets/hdr/qm8k.hdr");

		_shader->use();
		_shader->setInt1(0, "irradianceMap");
		_shader->setInt1(1, "prefilterMap");
		_shader->setInt1(2, "brdfLUT");

		glActiveTexture(GL_TEXTURE8);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getIrradianceMap());

		glActiveTexture(GL_TEXTURE9);
		glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getPrefilterMap());

		glActiveTexture(GL_TEXTURE10);
		glBindTexture(GL_TEXTURE_2D, _ibl->getBRDFLUT());
	}

	void SceneView::ShadowPass() {
		float nearPlane = _camera->getNear();
		float farPlane = _camera->getFar();

		float cascadeNear[NUM_CASCADES];
		float cascadeFar[NUM_CASCADES];

		cascadeNear[0] = nearPlane;
		cascadeFar[0] = nearPlane + _cascadeSplits[0] * (farPlane);

		cascadeNear[1] = cascadeFar[0];
		cascadeFar[1] = nearPlane + _cascadeSplits[1] * (farPlane);

		for (int i = 0; i < NUM_CASCADES; i++) {
			_lightSpaceMatrixCascade[i] = LightSpaceMatrix(cascadeNear[i], cascadeFar[i]);

			int baseRes = 4096;
			int res = baseRes >> i;   // 4096, 2048
			glViewport(0, 0, res, res);

			glBindFramebuffer(GL_FRAMEBUFFER, _cascadeFBO[i]);
			glClear(GL_DEPTH_BUFFER_BIT);

			_shadowShader->use();
			_shadowShader->setMat4(_lightSpaceMatrixCascade[i], "lightSpaceMatrix");

			// checker plane
			if (_checkerPlane) {
				glm::mat4 model(1.0f);
				_shadowShader->setMat4(model, "model");
				_checkerPlane->render();
			}

			// main mesh
			if (_object && _object->getMesh()) {
				glm::mat4 model(1.0f);
				model = glm::translate(glm::mat4(1.0f), _object->getMesh()->_position);
				_shadowShader->setMat4(model, "model");
				_object->getMesh()->render();
			}
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void SceneView::SkyboxRender() {
		if (_skybox) {
			glm::mat4 view = _camera->getViewMatrix();
			glm::mat4 projection = _camera->getProjection();
			_skybox->render(view, projection);
		}
	}

	void gui::SceneView::updatePhysics(float dt)
	{
		// --- sanity checks ---
		if (!_object || !_object->getMesh()) return;
		auto mesh = _object->getMesh();

		// constants
		const float floorY = planeHeight;           // same height as checkerboard
		const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

		// --- apply physics ---
		if (!mesh->_isStatic)
		{
			// apply gravity
			mesh->_acceleration = gravity;

			// integrate (basic semi-implicit Euler)
			mesh->_velocity += mesh->_acceleration * dt;
			mesh->_position += mesh->_velocity * dt;

			// --- floor collision clamp ---
			if (mesh->_position.y < floorY)
			{
				mesh->_position.y = floorY;
				mesh->_velocity.y = 0.0f;
			}
		}

		// optional: horizontal damping for stability
		mesh->_velocity.x *= 0.98f;
		mesh->_velocity.z *= 0.98f;
	}

/*
 * ------------------------------------------------
 *				KEYBOARD & MOUSE INPUT
 * ------------------------------------------------
 */

	void gui::SceneView::handleContinuousMovement(GLFWwindow* window, float dt) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		float kspd = 2.5f * dt;

		if (elements::Input::IsKeyPressed(window, GLFW_KEY_W)) {
			processMovementKey(GLFW_KEY_W, kspd);
		}
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_S)) {
			processMovementKey(GLFW_KEY_S, kspd);
		}
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_A)) {
			processMovementKey(GLFW_KEY_A, kspd);
		}
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_D)) {
			processMovementKey(GLFW_KEY_D, kspd);
		}
	}

	void gui::SceneView::resetMouseDelta() {
		// force next sample to re-seed
		// (reuse your firstMouse flag or equivalent)
		// simplest: store a boolean
		_firstMouse = true;  // make this a member instead of static
	}

	void gui::SceneView::handleMouseLook(GLFWwindow* window, double xpos, double ypos) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		// If cursor is not captured, only rotate when hovering
		bool captured = false;
		if (auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window)))
			captured = win->isMouseCaptured();

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

		if (_controlMode == ControlMode::Camera) {
			_camera->processMouseMovement(xoffset, yoffset);
		}
		else if (_controlMode == ControlMode::Object && _object) {
			_object->onMouseMove(xpos, ypos, elements::eInputButton::Right);
		}
	}

	void gui::SceneView::processMovementKey(int key, float delta) {
		if (_controlMode == ControlMode::Camera) {
			_camera->processKeyboard(key, delta);
		}
		else if (_controlMode == ControlMode::Object && _mesh) {
			// Object movement logic to be added here!
		}
	}

	elements::Camera* SceneView::getCamera() { return _camera.get(); }
}