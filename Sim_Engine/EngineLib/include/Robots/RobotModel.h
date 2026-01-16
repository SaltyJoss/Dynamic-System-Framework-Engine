#pragma once

// =============================================
//            File: RobotModel.h
// =============================================
// Structs representing a robotic model with links and joints.
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
			joints[i].angle = a;
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


