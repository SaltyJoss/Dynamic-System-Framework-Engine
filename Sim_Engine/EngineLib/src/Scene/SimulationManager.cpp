
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

#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/MeshLoader.h"
#include "Scene/Light.h"
#include "Scene/AxisOrientator.h"

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

	simManager::simManager() :
		_camera(nullptr), _frameBuffer(nullptr), _shaderBasic(nullptr), _shaderLit(nullptr), _shaderPBR(nullptr),
		_light(nullptr), _worldGridShader(nullptr), _shadowShader(nullptr), _size(3840, 2160)
	{
		_frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
		_frameBuffer->createBuffers(3840, 2160);

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

		_light = std::make_unique<scene::Light>();
		_sunLight = std::make_unique<scene::Light>();
		_sunLight->_isDirectional = true;
		_sunLight->setDirection(glm::vec3(-1.0f, -0.3f, 0.2f));
		_sunLight->_intensity = 1.0f;

		_camera = std::make_unique<scene::Camera>(glm::vec3(0.0f, 2.0f, 5.0f), 70.0f, static_cast<float>(_size.x) / static_cast<float>(_size.y), 0.1f, 1000.0f);
		_axisOrientator = std::make_unique<gui::AxisOrientator>();

		glGenVertexArrays(1, &_worldGridVAO);

		_mesh = std::make_shared<scene::Mesh>();
		_mesh->init();

		_physics = std::make_unique<physics::PhysicsSystem>();

		if (_checkerPlane) _checkerPlane->clean();
		_checkerPlane = createCheckerPlane(50.0f);
		planeY = 2.5f;

		InitShadowResource();
		InitIBL();
	}

	simManager::~simManager()
	{
		if (_frameBuffer) _frameBuffer->deleteBuffers();
		if (_mesh) _mesh->clean();
		if (_checkerPlane) _checkerPlane->clean();
	}

// --------------------------------------------------
//				    LIGHT & SKYBOX
// --------------------------------------------------
	void simManager::loadNewHDR(const std::string& path)
	{
		LOG_INFO("Loading new HDR: %s", path.c_str());
		D_INFO("Loading new HDR: %s", path.c_str());

		// Make sure IBL system exists
		if (!_ibl) {
			LOG_ERROR("Cannot load HDR because IBL system is not initialised.");
			D_FAIL("Cannot load HDR because IBL system is not initialised.");
			return;
		}

		_ibl->init(path); // rebuild envCubemap, irradiance, prefilter, brdfLUT
		D_SUCCESS("IBL rebuilt successfully.");

		// Update skybox
		_skybox->setEnvironmentTexture(_ibl->getEnvCubemap());

		LOG_INFO("HDR updated successfully.");
		D_SUCCESS("Loaded HDR successfully.");
	}

// --------------------------------------------------
//				CONTROL MODES & CAMERA
// --------------------------------------------------
	scene::Camera* simManager::getCamera() { return _camera.get(); }
	void simManager::resetView() { _camera->reset(); }

	void simManager::attachCameraToObject(scene::Object* obj) {
		if (!obj) return;

		_cameraFollowTarget = obj;

		glm::vec3 pos = obj->transform.position;
		glm::vec3 rot = obj->transform.rotation;

		_camera->startFollow(pos, rot, glm::vec3(0, 2, 5)); // example offset
	}

	void simManager::detachCameraFromObject() {
		_cameraFollowTarget = nullptr;
		_camera->clearFollow();
	}

	void simManager::oreintationGizmoRender() {
		// Placeholder for orientation gizmo rendering
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

			// initialise physics state
			obj->state.theta = Eigen::Vector3d::Zero();
			obj->state.angularVelocity = Eigen::Vector3d::Zero();
			obj->state.linearVelocity = Eigen::Vector3d::Zero();
			obj->state.mass = 1.0;
			obj->state.damping = 0.0;
			obj->state.inertia = Eigen::Matrix3d::Identity();
			obj->state.forces = Eigen::Vector3d::Zero();
			obj->state.torques = Eigen::Vector3d::Zero();

			_selectedObject = obj.get();
			_objects.push_back(std::move(obj));
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
			_objects.push_back(std::move(obj));
			result.push_back(raw);
		}

		return result;
	}

	std::shared_ptr<scene::Mesh> gui::simManager::createCheckerPlane(float size) {

		auto plane = std::make_shared<scene::Mesh>();

		std::vector<glm::vec3> pos = {
			{-size, planeHeight, -size},
			{ size, planeHeight, -size},
			{ size, planeHeight,  size},
			{-size, planeHeight,  size}
		};

		glm::vec3 normal(0.0f, 1.0f, 0.0f);

		for (auto& p : pos) {
			scene::VertexHolder vh(p, normal);
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

	void simManager::deleteObject(int index) {
		if (index < 0 || index >= _objects.size()) return;

		if (_selectedObject == _objects[index].get()) {
			_selectedObject = nullptr;
		}

		_objects.erase(_objects.begin() + index);
	}

// --------------------------------------------------
//				RENDERING ENTRY POINTS
// --------------------------------------------------
	void simManager::render() {
		updatePhysics(dt);
		_fpsCounter.update();
		ShadowPass();

		_frameBuffer->bind();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		glm::mat4 view = _camera->getViewMatrix();

		if (_hasRobot) {
			glm::mat4 base = glm::mat4(1.0f);
			base = glm::rotate(glm::radians(-90.0f), glm::vec3(1, 0, 0)); // aligns base link vertically (REMEMBER TO USE IF ROBOT XYZ AXES DIFFERENTLY)
			updateRobotKinematics(base);
		}

		//WorldGridRender();
		MeshRender();

		if (skyboxEnabled)
		{
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);

			SkyboxRender();

			glDepthMask(GL_TRUE);
			glDepthFunc(GL_LESS);
		}

		_axisOrientator->render(view);

		_frameBuffer->unbind();

		ImGui::Begin("Sim Engine");

		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);
		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image((void*)textureID, viewportPanelSize, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}

	void simManager::resize(int32_t width, int32_t height) {
		// ignore zero sizes
		if (width == 0 || height == 0) { return; }

		glViewport(0, 0, width, height);	// set OpenGL viewport
		_size = glm::ivec2(width, height);	// update internal size

		_frameBuffer->deleteBuffers();
		_frameBuffer->createBuffers(width, height);

		// update camera aspect ratio
		_camera->setAspect(static_cast<float>(width) / static_cast<float>(height));

		LOG_INFO("Resized simManager to %dx%d", width, height);
	}

// --------------------------------------------------
//						PHYSICS
// --------------------------------------------------
	void gui::simManager::updatePhysics(double dt) {
		// Update each object's physics state
		for (auto& obj : _objects) {
			if (obj) { _physics->update(dt, obj.get()); }
		}
	}

// --------------------------------------------------
//				  ROBOTIC ARM SYSTEM
// --------------------------------------------------
	// Method to load a robot model by name
	void simManager::loadRobot(const std::string& name) {
		clearRobot();

		std::string jsonPath = "Engine/assets/Objects/Robotic_Arm_Models/" + name + "/" + name + ".json";

		_robot = robots::RobotLoader::loadFromJSON(jsonPath);
		_hasRobot = true;

		{
			VecX q = _robot.makeJointVector();   // all angles at their defaults
			kinematics::Forward_Kinematics fk;

			mathlib::Pose T_ee = fk.FK(_robot.dhParams, q);

			LOG_INFO("FK zero config EE: x=%.4f y=%.4f z=%.4f",
				T_ee(0, 3), T_ee(1, 3), T_ee(2, 3));
			D_DEBUG("FK zero config EE: x=%.4f y=%.4f z=%.4f",
				T_ee(0, 3), T_ee(1, 3), T_ee(2, 3));
		}

		instantiateRobotLinks();
		buildLinkIndex();

		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());
	}

	// Method to create Object instances for each robot link
	void simManager::instantiateRobotLinks() {
		for (auto& link : _robot.links) {
			auto objs = loadMeshReturn(link.meshFile);
			if (objs.empty()) {
				LOG_ERROR("Failed to load mesh for link %s", link.name.c_str());
				D_ERROR("Failed to load mesh for link %s", link.name.c_str());
				continue;
			}
			scene::Object* obj = objs[0]; // assumes one object per link
			obj->transform.scale = glm::vec3(1.0f);
			obj->category = scene::ObjectCategory::RobotLink;
			link.attachedObject = obj;

			LOG_INFO_ONCE("Instantiated link: %s from %s", link.name.c_str(), link.meshFile.c_str());
			D_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
		}		
	}

	// Method to build a name-to-index map for robot links
	void simManager::buildLinkIndex() {
		_linkIndex.clear();
		for (size_t i = 0; i < _robot.links.size(); i++) {
			_linkIndex[_robot.links[i].name] = (int)i;
		}
	}

	// Method to update robot link transforms based on joint angles (NEEDS TO BE REVISED BASED ON ROBOT STRUCTURE)
	void simManager::updateRobotKinematics(const glm::mat4& baseTransform) {
		if (!_hasRobot) return;

		// Get current joint angles as Eigen vector
		VecX q = _robot.makeJointVector();

		// Compute forward kinematics
		kinematics::Forward_Kinematics fk;
		std::vector<Pose> eigenTrans = fk.linkTransforms(_robot.dhParams, q);

		// World transforms for each link
		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));

		// Convert Eigen poses to glm::mat4 and apply base transform to root link
		for (std::size_t i = 0; i < eigenTrans.size(); ++i) {
			const Pose& pose = eigenTrans[i];
			glm::mat4 glmMat(1.0f);
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) {
					glmMat[c][r] = static_cast<float>(pose(r, c));
				}
			}

			// Apply base transform to the root link
			if (i == 0) {
				world[i] = baseTransform * glmMat;
			} else {
				world[i] = glmMat;
			}
		}

		//int rootIdx = _linkIndex["link00"];  // Z1 root link (base static link)
		//world[rootIdx] = baseTransform;

		//// Sort joints in parent-to-child order
		//std::vector<RobotJoint> sorted = _robot.joints;

		//std::sort(sorted.begin(), sorted.end(),
		//	[&](const RobotJoint& a, const RobotJoint& b) {
		//		int a_parent_indx = _linkIndex[a.parent];
		//		int b_parent_indx = _linkIndex[b.parent];
		//		return a_parent_indx < b_parent_indx;
		//	});

		//for (auto& joint : sorted) {
		//	int parent = _linkIndex[joint.parent];
		//	int child = _linkIndex[joint.child];

		//	glm::mat4 T_offset = glm::translate(glm::mat4(1.0f), joint.offset * _robot.scale);
		//	glm::mat4 R_joint = glm::rotate(glm::mat4(1.0f), joint.angle, glm::normalize(joint.axis));

		//	world[child] = world[parent] * T_offset * R_joint;
		//}

		// Update link object transforms
		for (size_t i = 0; i < _robot.links.size(); i++) {
			auto* obj = _robot.links[i].attachedObject;
			auto* mesh = obj->getMesh();
			if (!mesh) continue;

			// Visual Scaling matrix
			glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(_robot.scale));

			// FK-driven world matrix goes straight into the mesh
			mesh->localTransform = world[i] * S;
		}
	}

	// Method to set the rotation angle of a specific robot link angle in degrees
	// NOTE: this sets the joint angle that affects the link, not the link transform directly
	void simManager::setRobotLinkRotation(const std::string& linkName, float angle) {
		if (!_hasRobot) {
			LOG_WARN_ONCE("No robot loaded to set link rotation.");
			D_WARN_ONCE("No robot loaded to set link rotation.");
			return;
		}
		auto it = _linkIndex.find(linkName);
		if (it == _linkIndex.end()) {
			D_ERROR_ONCE("Link name %s not found in robot model.", linkName.c_str());
			return;
		}
		int linkIdx = it->second;
		// Find the joint that connects to this link
		for (auto& joint : _robot.joints) {
			if (joint.child == linkName) {
				joint.angle = glm::radians(angle); // store angle in radians
				D_INFO_ONCE("%s -> %.2f degrees.", linkName.c_str(), angle);
				return;
			}
		}
		LOG_WARN_ONCE("No joint found for link %s to set rotation.", linkName.c_str());
		D_WARN_ONCE("No joint found for link %s to set rotation.", linkName.c_str());
	}

	// Method to clear the current robot from the scene
	void simManager::clearRobot() {
		if (!_hasRobot) return;

		// Remove robot objects from _objects
		for (auto& link : _robot.links) {
			if (link.attachedObject) {
				// find and erase matching object
				_objects.erase(
					std::remove_if(
						_objects.begin(),
						_objects.end(),
						[&](const std::unique_ptr<scene::Object>& obj) {
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
		D_WARN("old robot model destroyed");
	}

// --------------------------------------------------
//			 INTERNAL REDNDERING PIPELINE
// --------------------------------------------------
	void simManager::InitShadowResource() {
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

	void simManager::InitIBL()
	{
		_ibl = std::make_unique<render::IBL>();
		_ibl->init("Engine/assets/hdr/space-6.hdr");
	}

	void simManager::WorldGridRender() {
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

	void simManager::MeshRender() {
		shaders::Shader* shader = nullptr;

		switch (currentShaderMode) {
			case ShaderMode::Basic:     
				shader = _shaderBasic.get();
				break;
			case ShaderMode::Lit:
				shader = _shaderLit.get();
				break;
			case ShaderMode::PBR:
				shader = _shaderPBR.get();
				break;
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
				shader->setVec3(_light->getColour(), "colour");
				break;

			case ShaderMode::Lit:
				// (IMPORTANT) mesh_lit.frag needs: albedo, lightPosition, lightColour, lightIntensity, camPos
				shader->setVec3(_light->getColour(), "albedo");
				shader->setVec3(glm::vec3(-4.0f, 20.0f, 12.0f), "lightPosition");
				shader->setVec3(glm::vec3(1.0f, 0.95f, 0.9f), "lightColour");
				shader->setFlt1(1.0f, "lightIntensity");
				shader->setVec3(_camera->getPosition(), "camPos");
				break;

			case ShaderMode::PBR:
				shader->setVec3(_light->getColour(), "albedo");
				shader->setFlt1(0.0f, "metallic");
				shader->setFlt1(0.5f, "roughness");
				shader->setFlt1(1.0f, "ao");

				//shader->setVec3(glm::normalize(_light->getDirection()), "lightDirection");
				shader->setVec3(glm::vec3(1.0f, 0.95f, 0.9f), "lightColour");
				shader->setFlt1(_light->getIntensity(), "lightIntensity");
				shader->setVec3(_camera->getPosition(), "camPos");

				shader->setInt1(0, "irradianceMap");
				shader->setInt1(1, "prefilterMap");
				shader->setInt1(2, "brdfLUT");

				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getIrradianceMap());

				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_CUBE_MAP, _ibl->getPrefilterMap());

				glActiveTexture(GL_TEXTURE2);
				glBindTexture(GL_TEXTURE_2D, _ibl->getBRDFLUT());

				LOG_INFO_ONCE("RadianceMap = %u, Prefilter = %u, BRDF = %u", _ibl->getIrradianceMap(), _ibl->getPrefilterMap(), _ibl->getBRDFLUT());
				D_INFO_ONCE("RadianceMap = %u, Prefilter = %u, BRDF = %u", _ibl->getIrradianceMap(), _ibl->getPrefilterMap(), _ibl->getBRDFLUT());
				break;
			}

			int loc = glGetUniformLocation(shader->getProgramID(), "albedo");
			LOG_INFO_ONCE("Lit Shader albedo uniform location = %d", loc);

			//obj->getMesh()->update(shader);
			obj->getMesh()->render();
		}
	}


	void simManager::ShadowPass() {
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

	glm::mat4 simManager::LightSpaceMatrix(float nearPlane, float farPlane) {
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


	void simManager::SkyboxRender() {
		glm::mat4 view = _camera->getViewMatrix();
		glm::mat4 projection = _camera->getProjection();

		_skybox->setEnvironmentTexture(_ibl->getEnvCubemap());
		_skybox->render(projection, view);
	}

	void simManager::reloadAllShaders()
	{
		_shaderBasic->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_basic.frag.glsl");
		_shaderLit->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_lit.frag.glsl");
		_shaderPBR->load("Engine/assets/shaders/vs_pbr.vert.glsl", "Engine/assets/shaders/mesh_pbr.frag.glsl");

		LOG_INFO("All shaders reloaded from disk.");
		D_INFO_ONCE("All shaders reloaded from disk.");
	}

// --------------------------------------------------
//					INPUT HANDLING
// --------------------------------------------------
	void gui::simManager::processMovementKey(int key, float delta) {
		if (ctrlMode == ControlMode::Camera) {
			_camera->processKeyboard(key, delta);
		}
		else if (ctrlMode == ControlMode::Object && _mesh) {
			// WILL ADD OBJECT MOVEMENT LATER
		}
	}

	void gui::simManager::handleContinuousMovement(GLFWwindow* window, float dt) {
		auto* win = static_cast<window::GLWindow*>(glfwGetWindowUserPointer(window));
		if (!win || !win->isMouseCaptured()) return;

		float kspd = 2.5f * dt;

		// Forward
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_W)) {
			processMovementKey(GLFW_KEY_W, kspd);
		}
		// Backward
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_S)) {
			processMovementKey(GLFW_KEY_S, kspd);
		}
		// Left
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_A)) {
			processMovementKey(GLFW_KEY_A, kspd);
		}
		// Right
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_D)) {
			processMovementKey(GLFW_KEY_D, kspd);
		}
		// Up
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_SPACE)) {
			processMovementKey(GLFW_KEY_SPACE, kspd);
		}
		// Down
		if (scene::Input::IsKeyPressed(window, GLFW_KEY_LEFT_SHIFT)) {
			processMovementKey(GLFW_KEY_LEFT_SHIFT, kspd);
		}
	}

	void gui::simManager::handleMouseLook(GLFWwindow* window, double xpos, double ypos) {
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
			_selectedObject->onMouseMove(xpos, ypos, scene::eInputButton::Right);
		}
	}


	void simManager::onMouseMove(double x, double y, scene::eInputButton button) {
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

	void simManager::onMouseWheel(double delta) {
		auto* obj = _selectedObject;
		if (!_isHovered) return;

		if (ctrlMode == ControlMode::Camera) _camera->onMouseWheel(delta);
		else if (ctrlMode == ControlMode::Object && _mesh) obj->transform.position.z += (float)delta * 0.1f;
	}


	void gui::simManager::resetMouseDelta() { _firstMouse = true; }
}