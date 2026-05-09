// DSFE_GUI RobotRenderer.cpp
#include "Robots/RobotRenderer.h"
#include "Robots/RobotPresentationBuilder.h"
#include "Robots/RobotModel.h"

#include <glm/glm.hpp>

#include "Scene/Mesh.h"
#include "Scene/Object.h"

#include <filesystem>
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"

namespace fs = std::filesystem;

static glm::quat q_corr = glm::angleAxis(glm::radians(90.0f), glm::vec3(1, 0, 0));

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

// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
static mathlib::Quat rpyRadToQuat(const mathlib::Vec3& rpyRad) {
	const double roll = rpyRad.x();
	const double pitch = rpyRad.y();
	const double yaw = rpyRad.z();

	const Quat qx(Eigen::AngleAxisd(roll, Vec3(1.0, 0.0, 0.0)));
	const Quat qy(Eigen::AngleAxisd(pitch, Vec3(0.0, 1.0, 0.0)));
	const Quat qz(Eigen::AngleAxisd(yaw, Vec3(0.0, 0.0, 1.0)));

	return (qz * qy * qx).normalized();
}

void RobotRenderer::bind(const RobotRenderBinding& binding) {
	linkRenderMap.clear();

	LOG_INFO("Link render map cleared");
	LOG_INFO("binding entries = %zu", binding.linkVisuals.size());

	for (const auto& [linkName, visuals] : binding.linkVisuals) {
		LinkRenderData renderData;

		for (auto* obj : visuals) {
			renderData.visuals.push_back(obj);
		}

		linkRenderMap[linkName] = renderData;
	}
}

// Method to apply the computed world transforms to the corresponding Object instances for each robot link
void RobotRenderer::applyTransforms(const robots::RobotModel& robot, const std::vector<mathlib::Mat4>& world) {
	const bool isAligned = robot.baseFrameIsEngineAligned;
	const size_t n = robot.links.size();

	for (int i = 0; i < n; ++i) {
		const auto& link = robot.links[i];

		auto it = linkRenderMap.find(link.name);
		if (it == linkRenderMap.end()) { continue; }

		glm::mat4 T = toGlm(world[i]);
		glm::vec3 pos = glm::vec3(T[3]); // Extract translation from the 4x4 matrix
		glm::quat q = glm::quat_cast(T); // Extract rotation as a quaternion

		glm::quat q_rot = isAligned ? (q * q_corr) : q;

		for (auto* obj : it->second.visuals) {
			if (!obj) { continue; }

			obj->transform.position = pos;
			obj->transform.rotQ = q_rot;
		}
	}
}

// Method to clear the current robot from the scene
void RobotRenderer::clearRobotModel(const robots::RobotModel& robot) {
	// Remove robot objects from _objects
	for (auto& link : robot.links) {
		auto it = linkRenderMap.find(link.name);
		if (it == linkRenderMap.end()) continue;
		
	}
	D_WARN("Old robot model removed");
}