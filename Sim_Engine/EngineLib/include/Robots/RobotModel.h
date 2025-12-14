#pragma once

// =============================================
//            File: RobotModel.h
// =============================================
// Structs representing a robotic model with links and joints.
//
// Summary:
// =============================================
//
// structs:
// --------------------------------------------
// RobotLink
//      -> Represents a single link in the robotic model, including its name, mesh file, and attached object.
// RobotJoint
//      -> Represents a joint connecting two links, including its name, parent and child links, axis of rotation, offset, and current angle.
// RobotModel
//      -> Represents the entire robotic model, including scale, links, and joints.
// --------------------------------------------
// 
// Internal State Variables:
// --------------------------------------------
// RobotLink:
// ---
// std::string name
//      -> Name of the robotic link.
// std::string meshFile
//      -> File path to the mesh associated with the link.
// scene::Object* attachedObject
// 		-> Pointer to the object attached to this link (initialized to nullptr).
// 
// RobotJoint:
// ---
// std::string name
//      -> Name of the robotic joint.
// std::string parent
//      -> Name of the parent link connected by this joint.
// std::string child
//	     -> Name of the child link connected by this joint.
// glm::vec3 axis
// 		-> Axis of rotation for the joint.
// glm::vec3 offset
//      -> Offset vector from the parent link to the joint.
// float angle
//      -> Current angle of the joint (initialized to 0.0f).
// 
// RobotModel:
// ---
// float scale
//		-> Scale factor for the entire robotic model (initialized to 1.0f).
// std::vector<RobotLink> links
//		-> Vector of links that make up the robotic model.
// std::vector<RobotJoint> joints
//      -> Vector of joints that connect the links in the robotic model.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <Kinematics/DH_Params.h>
#include "Scene/Object.h"

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

extern ENGINE_API Debug gLog;

struct RobotLink {
	std::string name;
	std::string meshFile;
	scene::Object* attachedObject = nullptr;
};

struct RobotJoint {
	std::string name;
	std::string parent;
	std::string child;
	
	glm::vec3 axis;
	glm::vec3 offset;
	float angle = 0.0f;

	float minAngle; // lower limit 
	float maxAngle; // upper limit 
	bool continuous; // true for base, false for limited joints
};

struct RobotModel {
	float scale = 1.0f;
	std::vector<RobotLink> links;
	std::vector<RobotJoint> joints;

	std::vector<kinematics::DH_Params> dhParams; // optional DH parameters for kinematics

	// Create an Eigen vector of joint angles
	VecX makeJointVector() const {
		const int n = static_cast<int>(joints.size());
		VecX q(n);
		for (int i = 0; i < n; ++i) {
			q(i) = static_cast<double>(joints[i].angle);
		}
		return q;
	}

	// Set joint angles from an Eigen vector
	void setJointVector(const VecX& q) {
		const int n = static_cast<int>(joints.size());
		if (q.size() != n) {
			ERROR("Joint vector size mismatch: expected %d, got %d", n, q.size());
			D_ERROR("Joint vector size mismatch: expected %d, got %d", n, q.size());
			return;
		}
		for (int i = 0; i < n; ++i) {
			float a = static_cast<float>(q(i));
			joints[i].angle = static_cast<float>(joints[i], a);
		}
	}
};

inline float wrapRad(float a) {
	const float TWO_PI = 6.28318530718f;
	a = fmod(a, TWO_PI);
	if (a < 0.0f) a += TWO_PI;
	return a;
}

inline float clampJointAngle(const RobotJoint& joint, float angle) {
	if (joint.continuous) {
		return wrapRad(angle);
	}

	return std::clamp(angle, joint.minAngle, joint.maxAngle);
}


