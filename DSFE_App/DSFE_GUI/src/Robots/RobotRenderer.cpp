// DSFE_GUI RobotRenderer.cpp
#include "Robots/RobotRenderer.h"
#include "Robots/RobotModel.h"

#include <glm/glm.hpp>

#include "Scene/Mesh.h"
#include "Scene/Object.h"

#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

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

// Method to create Object instances for each robot link
void RobotRenderer::instantiateRobotLinks(const robots::RobotModel& robot) {
	linkRenderMap.clear();
	for (auto& link : robot.links) {
		LinkRenderData renderData;

		// Per-mesh material entries (new format with meshEntries)
		if (!link.visual.meshEntries.empty()) {
			for (const auto& entry : link.visual.meshEntries) {
				const auto fullPath = (paths::assets() / "objects" / "Robotic_Arm_Models" / entry.meshFile).string();
				auto objs = _loadMeshReturn(fullPath);

				// If no meshes were loaded for this entry, skip it
				for (auto* obj : objs) {
					if (scene::Mesh* mesh = obj->getMesh()) {
						const Vec4& rgba = entry.hasMaterial
							? entry.material
							: Vec4(0.7, 0.0, 0.2, 1.0); // Default material if not specified

						mesh->setAlbedo(glm::vec3(rgba.x(), rgba.y(), rgba.z()));
						mesh->setMetallic(entry.hasMaterial ? entry.metallic : 0.5f);
						mesh->setRoughness(entry.hasMaterial ? entry.roughness : 0.5f);
						mesh->rebuildGPU();
					}

					obj->name = link.name;
					obj->category = scene::ObjectCategory::RobotLink;
					obj->transform.scale = glm::vec3(robot.scale);

					renderData.visuals.push_back(obj);
				}
			}

			linkRenderMap[link.name] = renderData;
			continue;
		}

		// If no mesh entries, fall back to legacy single mesh or multiple mesh files
		std::vector<scene::Object*> objs;

		// Legacy Mesh Path Support
		if (!link.visual.meshFiles.empty()) {
			for (const auto& meshRelPath : link.visual.meshFiles) {
				const auto fullPath = (paths::assets() / "objects" / "Robotic_Arm_Models" / meshRelPath).string();
				auto partObjs = _loadMeshReturn(fullPath);
				objs.insert(objs.end(), partObjs.begin(), partObjs.end());
			}
		}
		else {
			continue;
		}

		// If no meshes were loaded, skip this link
		if (objs.empty()) {
			LOG_WARN("No meshes found for link %s", link.name.c_str());
			continue;
		}

		// Merge multiple meshes into one Object (if necessary)
		scene::Object* rootObj = objs[0];
		scene::Mesh* baseMesh = rootObj->getMesh();

		// If there are multiple meshes (e.g., from a multi-part OBJ), merge them into the first one
		for (size_t i = 1; i < objs.size(); ++i) {
			if (auto* extra = objs[i]->getMesh()) {
				if (baseMesh) { baseMesh->appendGeometry(*extra); }
			}

			objs[i]->name.clear();
			objs[i]->category = scene::ObjectCategory::General;
		}

		if (baseMesh) {
			baseMesh->rebuildGPU();
		}

		for (auto* obj : objs) {
			obj->name = link.name;
			obj->category = scene::ObjectCategory::RobotLink;
			obj->transform.scale = glm::vec3(robot.scale);

			renderData.visuals.push_back(obj);
			linkRenderMap[link.name] = renderData;
		}
	}

	LOG_INFO_ONCE("Instantiated %zu robot links", robot.links.size());
}

// Method to apply the computed world transforms to the corresponding Object instances for each robot link
void RobotRenderer::applyTransforms(const robots::RobotModel& robot, const std::vector<mathlib::Mat4>& world) {
	for (int i = 0; i < robot.links.size(); ++i) {
		const auto& link = robot.links[i];
		auto it = linkRenderMap.find(link.name);
		if (it == linkRenderMap.end()) continue;

		glm::mat4 T = toGlm(world[i]);

		for (auto* obj : it->second.visuals) {
			obj->transform.position = glm::vec3(T[3]);
			obj->transform.rotQ = glm::quat_cast(T);
		}
	}
}

//// Method to clear the current robot from the scene
//void RobotRenderer::clearRobot() {
//	if (!_hasRobot) return;
//
//	// Remove robot objects from _objects
//	for (auto& link : _robot.links) {
//		for (auto* dead : link.attachedObjects) {
//			if (!dead) continue;
//			_objects.erase(
//				std::remove_if(_objects.begin(), _objects.end(),
//					[&](const std::unique_ptr<scene::Object>& obj) { return obj.get() == dead; }),
//				_objects.end()
//			);
//		}
//		link.attachedObjects.clear();
//		link.attachedObject = nullptr;
//	}
//
//	_robot.links.clear();
//	_robot.joints.clear();
//	_linkIndex.clear();
//	_hasRobot = false;
//
//	D_WARN("Old robot model removed");
//}