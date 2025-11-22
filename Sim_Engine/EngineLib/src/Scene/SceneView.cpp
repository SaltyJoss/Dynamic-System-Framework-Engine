
#include "pch.h"
#include "Scene/Object.h"
#include "Scene/SceneView.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtx/euler_angles.hpp>  
#include <imgui.h>

#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/MeshLoader.h"
#include "Scene/Light.h"

#include "Physics/PhysicsSystem.h"
#include "Robots/RobotLoader.h"
#include "Robots/RobotModel.h"

#include "Rendering/SkyboxRenderer.h"
#include "Rendering/ShaderUtil.h"
#include "Rendering/OpenGLBufferManager.h"
#include "Rendering/IBL.h"
#include "Rendering/Texture.h"

#include <Platform/WindowManager.h>

#include "EngineLib/LogMacros.h"

namespace gui{
// --------------------------------------------------
//				CONSTRUCTOR & DESTRUCTOR
// --------------------------------------------------

	SceneView::SceneView() :
		_camera(nullptr), _frameBuffer(nullptr), _shaderBasic(nullptr), _shaderLit(nullptr), _shaderPBR(nullptr),
		_light(nullptr), _worldGridShader(nullptr), _shadowShader(nullptr), _size(1920, 1080)
	{
		_frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
		_frameBuffer->createBuffers(1920, 1080);

		// Shader Types A
		_shaderBasic = std::make_shared<shaders::Shader>();
		_shaderBasic->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_basic.frag.glsl");

		_shaderLit = std::make_shared<shaders::Shader>();
		_shaderLit->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_lit.frag.glsl");

		_shaderPBR = std::make_shared<shaders::Shader>();
		_shaderPBR->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_pbr.frag.glsl");

		_skybox = std::make_unique<render::SkyboxRenderer>();
		
		// Shader Types B
		_worldGridShader = std::make_unique<shaders::Shader>();
		_worldGridShader->load("Engine/assets/shaders/world_grid.vert.glsl", "Engine/assets/shaders/world_grid.frag.glsl");

		_shadowShader = std::make_unique<shaders::Shader>();
		_shadowShader->load("Engine/assets/shaders/shadow_depth.vert.glsl", "Engine/assets/shaders/shadow_depth.frag.glsl");

		_light = std::make_unique<elements::Light>();
		_sunLight = std::make_unique<elements::Light>();
		_sunLight->_isDirectional = true;
		_sunLight->setDirection(glm::vec3(-1.0f, -0.3f, 0.2f));
		_sunLight->_intensity = 1.0f;

		_camera = std::make_unique<elements::Camera>(glm::vec3(0, 15, 20), 45.0f, 16 / 9, 0.5f, 2000.0f);

		glGenVertexArrays(1, &_worldGridVAO);

		_mesh = std::make_shared<elements::Mesh>();
		_mesh->init();

		_physics = std::make_unique<physics::PhysicsSystem>();

		if (_checkerPlane) _checkerPlane->clean();
		_checkerPlane = createCheckerPlane(50.0f);
		planeY = 2.5f;

		InitShadowResource();
		InitIBL();
	}

	SceneView::~SceneView()
	{
		if (_frameBuffer) _frameBuffer->deleteBuffers();
		if (_mesh) _mesh->clean();
		if (_checkerPlane) _checkerPlane->clean();
	}

// --------------------------------------------------
//				    LIGHT & SKYBOX
// --------------------------------------------------
	void SceneView::loadNewHDR(const std::string& path)
	{
		LOG_INFO("Loading new HDR: %s", path.c_str());

		// Make sure IBL system exists
		if (!_ibl) {
			LOG_ERROR("Cannot load HDR because IBL system is not initialised.");
			return;
		}

		_ibl->init(path); // rebuild envCubemap, irradiance, prefilter, brdfLUT

		// Update skybox
		_skybox->setEnvironmentTexture(_ibl->getEnvCubemap());

		LOG_INFO("HDR updated successfully.");
	}

// --------------------------------------------------
//				CONTROL MODES & CAMERA
// --------------------------------------------------
	elements::Camera* SceneView::getCamera() { return _camera.get(); }
	void SceneView::resetView() { _camera->reset(); }

	void SceneView::attachCameraToObject(elements::Object* obj) {
		if (!obj) return;

		_cameraFollowTarget = obj;

		glm::vec3 pos = obj->transform.position;
		glm::vec3 rot = obj->transform.rotation;

		_camera->startFollow(pos, rot, glm::vec3(0, 2, 5)); // example offset
	}

	void SceneView::detachCameraFromObject() {
		_cameraFollowTarget = nullptr;
		_camera->clearFollow();
	}


// --------------------------------------------------
//			    MESH LOADING & GEOMETRY
// --------------------------------------------------
	void SceneView::loadMesh(const std::string& filepath) {
		gui::MeshLoader loader;
		auto meshes = loader.load(filepath);

		if (meshes.empty()) {
			LOG_WARN("No meshes imported from %s", filepath.c_str());
			return;
		}

		// For now: spawn one Object per submesh
		for (auto& m : meshes) {
			auto obj = std::make_unique<elements::Object>(m);

			// initialise physics state
			obj->state.theta = Eigen::Vector3d::Zero();
			obj->state.angularVelocity = Eigen::Vector3d::Zero();
			obj->state.linearVelocity = Eigen::Vector3d::Zero();
			obj->state.mass = 1.0;
			obj->state.inertia = Eigen::Matrix3d::Identity();
			obj->state.forces = Eigen::Vector3d::Zero();
			obj->state.torques = Eigen::Vector3d::Zero();

			_selectedObject = obj.get();
			_objects.push_back(std::move(obj));
		}

		LOG_INFO("Loaded %zu submeshes from %s", meshes.size(), filepath.c_str());
	}

	std::vector<elements::Object*> SceneView::loadMeshReturn(const std::string& filepath) {
		gui::MeshLoader loader;
		auto meshes = loader.load(filepath);

		std::vector<elements::Object*> result;

		for (auto& m : meshes) {
			auto obj = std::make_unique<elements::Object>(m);
			auto raw = obj.get();
			_objects.push_back(std::move(obj));
			result.push_back(raw);
		}

		return result;
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

// --------------------------------------------------
//				RENDERING ENTRY POINTS
// --------------------------------------------------
	void SceneView::render() {
		updatePhysics(0.00833f); // temp fixed timestep at 120fps
		_fpsCounter.update();
		ShadowPass();

		_frameBuffer->bind();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		if (_hasRobot) {
			glm::mat4 base = glm::mat4(1.0f);
			base = glm::rotate(glm::radians(-90.0f), glm::vec3(1, 0, 0)); // aligns base link vertically (REMEMBER TO USE IF ROBOT XYZ AXES DIFFERENTLY)
			updateRobotKinematics(base);
		}

		WorldGridRender();
		MeshRender();

		if (skyboxEnabled)
		{
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);

			SkyboxRender();

			glDepthMask(GL_TRUE);
			glDepthFunc(GL_LESS);
		}

		_frameBuffer->unbind();

		ImGui::Begin("Sim Engine");

		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);
		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image((void*)textureID, viewportPanelSize, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}

	void SceneView::resize(int32_t width, int32_t height) {
		// ignore zero sizes
		if (width == 0 || height == 0) { return; }

		// update camera projection
		float aspect = width / height;
		_camera->setAspect(aspect);

		// store updated size
		_size = glm::vec2(width, height);

		// rebuild framebuffer
		_frameBuffer->deleteBuffers();
		_frameBuffer->createBuffers(width, height);

		LOG_INFO("Framebuffer resized: %d x %d", width, height);
	}

// --------------------------------------------------
//						PHYSICS
// --------------------------------------------------
	void gui::SceneView::updatePhysics(double dt) {
		// Update each object's physics state
		for (auto& obj : _objects) {
			if (obj) { _physics->update(dt, obj.get()); }
		}
	}

// --------------------------------------------------
//				  ROBOTIC ARM SYSTEM
// --------------------------------------------------
	// Method to load a robot model by name
	void SceneView::loadRobot(const std::string& name) {
		clearRobot();

		std::string jsonPath = "Engine/assets/Objects/Robotic_Arm_Models/" + name + "/" + name + ".json";

		_robot = robots::RobotLoader::loadFromJSON(jsonPath);
		_hasRobot = true;

		instantiateRobotLinks();
		buildLinkIndex();

		LOG_INFO("Loaded robot: %s", name.c_str());
	}

	// Method to create Object instances for each robot link
	void SceneView::instantiateRobotLinks() {


		for (auto& link : _robot.links) {
			auto objs = loadMeshReturn(link.meshFile);
			if (objs.empty()) {
				LOG_ERROR("Failed to load mesh for link %s", link.name.c_str());
				continue;
			}
			elements::Object* obj = objs[0]; // assumes one object per link
			obj->transform.scale = glm::vec3(_robot.scale);
			link.attachedObject = obj;

			LOG_INFO("Instantiated link: %s from %s", link.name.c_str(), link.meshFile.c_str());
		}
		LOG_INFO("Instantiated %zu robot links", _robot.links.size());
	}

	// Method to build a name-to-index map for robot links
	void SceneView::buildLinkIndex() {
		_linkIndex.clear();
		for (size_t i = 0; i < _robot.links.size(); i++) {
			_linkIndex[_robot.links[i].name] = (int)i;
		}
	}

	void SceneView::updateRobotKinematics(const glm::mat4& baseTransform) {
		if (!_hasRobot) return;

		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));

		int rootIdx = _linkIndex["link00"];  // Z1 root link
		world[rootIdx] = baseTransform;

		// Sort joints in parent-to-child order
		std::vector<RobotJoint> sorted = _robot.joints;

		std::sort(sorted.begin(), sorted.end(),
			[&](const RobotJoint& a, const RobotJoint& b) {
				int a_parent_indx = _linkIndex[a.parent];
				int b_parent_indx = _linkIndex[b.parent];
				return a_parent_indx < b_parent_indx;
			});

		for (auto& joint : sorted) {
			int parent = _linkIndex[joint.parent];
			int child = _linkIndex[joint.child];

			glm::mat4 T_offset = glm::translate(glm::mat4(1.0f), joint.offset * _robot.scale);
			glm::mat4 R_joint = glm::rotate(glm::mat4(1.0f), joint.angle, glm::normalize(joint.axis));

			world[child] = world[parent] * T_offset * R_joint;
		}

		// Update link object transforms
		for (size_t i = 0; i < _robot.links.size(); i++) {
			auto* obj = _robot.links[i].attachedObject;
			auto* mesh = obj->getMesh();
			if (!mesh) continue;

			// Robot-object transform is identity
			obj->transform.position = glm::vec3(0.0f);
			obj->transform.rotation = glm::vec3(0.0f);
			obj->transform.scale = glm::vec3(1.0f);

			// Visual scale if you need the robot bigger/smaller
			float s = _robot.scale;
			glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(s));

			// FK-driven world matrix goes straight into the mesh
			mesh->localTransform = world[i] * S;

			auto pos = glm::vec3(world[i][3]);
		}
	}

	// Method to clear the current robot from the scene
	void SceneView::clearRobot() {
		if (!_hasRobot) return;

		// Remove robot objects from _objects
		for (auto& link : _robot.links) {
			if (link.attachedObject) {
				// find and erase matching object
				_objects.erase(
					std::remove_if(
						_objects.begin(),
						_objects.end(),
						[&](const std::unique_ptr<elements::Object>& obj) {
							return obj.get() == link.attachedObject;
						}),
					_objects.end()
				);
			}
		}

		_robot.links.clear();
		_robot.joints.clear();
		_linkIndex.clear();
		_hasRobot = false;

		LOG_INFO("Cleared old robot model");
	}

// --------------------------------------------------
//			 INTERNAL REDNDERING PIPELINE
// --------------------------------------------------
	void SceneView::InitShadowResource() {
		int shadowRes[NUM_CASCADES] = { 4096, 4096 };

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
		_ibl->init("Engine/assets/hdr/space-6.hdr");
	}

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
		shaders::Shader* shader = nullptr;

		switch (currentShaderMode) {
			case ShaderMode::Basic:     shader = _shaderBasic.get(); break;
			case ShaderMode::Lit:       shader = _shaderLit.get(); break;
			case ShaderMode::PBR:       shader = _shaderPBR.get(); break;
		}

		if (!shader) {
			LOG_ERROR("Shader is NULL after switch!");
			return;
		}

		shader->use();

		// Only PBR know about cascades & those uniforms
		if (currentShaderMode == ShaderMode::PBR) {
			for (int i = 0; i < NUM_CASCADES; i++) {
				glActiveTexture(GL_TEXTURE5 + i);
				glBindTexture(GL_TEXTURE_2D, _cascadeDepth[i]);
				shader->setInt1(5 + i, "cascadeShadowMap[" + std::to_string(i) + "]");
				shader->setMat4(_lightSpaceMatrixCascade[i], "lightSpaceMatrix[" + std::to_string(i) + "]");
			}

			shader->setFlt2(_cascadeSplits[0], _cascadeSplits[1], "cascadeSplits");
		}

		// Camera / SunLight / light common to all mesh shaders
		_camera->update(shader);
		_sunLight->update(shader);
		_light->update(shader);

		for (auto& obj : _objects) {
			if (!obj || !obj->getMesh()) continue;

			if (_cameraFollowTarget == obj.get()) {
				_camera->setFollowTarget(
					obj->transform.position,
					obj->transform.rotation
				);
			}

			glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;
			shader->setMat4(model, "model");
			shader->setBool(false, "isFloor");

			// Per-mode material uniforms
			switch (currentShaderMode)
			{
			case ShaderMode::Basic:
				// (IMPORTANT) mesh_basic.frag needs: uniform vec3 color;
				shader->setVec3(glm::vec3(0.8f, 0.3f, 0.2f), "color");
				break;

			case ShaderMode::Lit:
				// (IMPORTANT) mesh_lit.frag needs: albedo, lightPosition, lightColour, lightIntensity, camPos
				shader->setVec3(glm::vec3(0.8f, 0.3f, 0.2f), "albedo");
				shader->setVec3(_light->getPosition(), "lightPosition");
				shader->setVec3(_light->getColour(), "lightColour");
				shader->setFlt1(_light->getIntensity(), "lightIntensity");
				shader->setVec3(_camera->getPosition(), "camPos");
				break;

			case ShaderMode::PBR:
				shader->setVec3(glm::vec3(0.8f, 0.3f, 0.2f), "albedo");
				shader->setFlt1(0.0f, "metallic");
				shader->setFlt1(0.3f, "roughness");
				shader->setFlt1(1.0f, "ao");
				shader->setBool(false, "useTexture");

				shader->setInt1(0, "irradianceMap");
				shader->setInt1(1, "prefilterMap");
				shader->setInt1(2, "brdfLUT");

				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getIrradianceMap());

				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getPrefilterMap());

				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, _ibl->getBRDFLUT());
				break;
			}

			obj->getMesh()->update(shader);
			obj->getMesh()->render();
		}
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
			int res = baseRes >> i;   // 4096, 4096
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
			for (auto& obj : _objects) {
				if (!obj || !obj->getMesh()) continue;

				glm::mat4 model = obj->transform.toMatrix() * obj->getMesh()->localTransform;

				_shadowShader->setMat4(model, "model");
				obj->getMesh()->render();
			}
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
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

		// Compute cascade center in light space
		glm::vec3 center = {
			0.5f * (minX + maxX),
			0.5f * (minY + maxY),
			0.5f * (minZ + maxZ)
		};

		// Cascade radius (half-size of the bounding sphere)
		float radius = glm::length(glm::vec3(maxX - minX, maxY - minY, 0.0f)) * 0.5f;

		// Pick resolution based on cascade level (match your ShadowPass())
		// (ShadowPass uses 4096 >> index, so we assume highest = 4096)
		int shadowMapResolution = 4096;

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


	void SceneView::SkyboxRender() {
		glm::mat4 view = _camera->getViewMatrix();
		glm::mat4 projection = _camera->getProjection();

		_skybox->setEnvironmentTexture(_ibl->getEnvCubemap());
		_skybox->render(projection, view);
	}

	void SceneView::reloadAllShaders()
	{
		_shaderBasic->reload();
		_shaderLit->reload();
		_shaderPBR->reload();
		_shaderPBRShadow->reload();

		LOG_INFO("All shaders reloaded from disk.");
	}

// --------------------------------------------------
//					INPUT HANDLING
// --------------------------------------------------
	void gui::SceneView::processMovementKey(int key, float delta) {
		if (ctrlMode == ControlMode::Camera) {
			_camera->processKeyboard(key, delta);
		}
		else if (ctrlMode == ControlMode::Object && _mesh) {
			// WILL ADD OBJECT MOVEMENT LATER
		}
	}

	void gui::SceneView::handleContinuousMovement(GLFWwindow* window, float dt) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		float kspd = 2.5f * dt;

		// Forward
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_W)) {
			processMovementKey(GLFW_KEY_W, kspd);
		}
		// Backward
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_S)) {
			processMovementKey(GLFW_KEY_S, kspd);
		}
		// Left
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_A)) {
			processMovementKey(GLFW_KEY_A, kspd);
		}
		// Right
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_D)) {
			processMovementKey(GLFW_KEY_D, kspd);
		}
		// Up
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_SPACE)) {
			processMovementKey(GLFW_KEY_SPACE, kspd);
		}
		// Down
		if (elements::Input::IsKeyPressed(window, GLFW_KEY_LEFT_SHIFT)) {
			processMovementKey(GLFW_KEY_LEFT_SHIFT, kspd);
		}
	}

	void gui::SceneView::handleMouseLook(GLFWwindow* window, double xpos, double ypos) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

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

		if (ctrlMode == ControlMode::Camera) {
			_camera->processMouseMovement(xoffset, yoffset);
		}
		else if (ctrlMode == ControlMode::Object && _selectedObject) {
			_selectedObject->onMouseMove(xpos, ypos, elements::eInputButton::Right);
		}
	}


	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) {
		glm::vec2 pos2d{ x, y };
		glm::vec2 delta = pos2d - _lastMousePos;
		_lastMousePos = pos2d;

		if (!_isHovered) {
			_camera->setCurrentPos2D(pos2d);
			_selectedObject->setLastMousePos(pos2d);
			return;
		}

		if (ctrlMode == ControlMode::Camera) {
			_camera->onMouseMove(x, y, button);
		}
		else if (ctrlMode == ControlMode::Object && _selectedObject) {
			_selectedObject->onMouseMove(x, y, button);
		}
	}

	void SceneView::onMouseWheel(double delta) {
		auto* obj = _selectedObject;
		if (!_isHovered) return;

		if (ctrlMode == ControlMode::Camera) _camera->onMouseWheel(delta);
		else if (ctrlMode == ControlMode::Object && _mesh) obj->transform.position.z += (float)delta * 0.1f;
	}


	void gui::SceneView::resetMouseDelta() { _firstMouse = true; }
}