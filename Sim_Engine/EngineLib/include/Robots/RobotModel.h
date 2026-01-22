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

namespace robots {
	struct RobotLink {
		std::string name = "";
		std::string meshFile = "";
		scene::Object* attachedObject = nullptr;

		// VISUAL-ONLY transform
		glm::vec3 visual_origin_xyz{ 0.0f, 0.0f, 0.0f };
		glm::quat visual_origin_rpy{ 1.0, 0.0, 0.0, 0.0 };

		// FINAL correction applied ONLY at render time (mesh frame -> link frame)
		glm::mat4 meshToLink = glm::mat4(1.0f);
	};

	struct RobotJoint {
		std::string name = "";
		std::string parent = "";
		std::string child = "";

		// URDF joint frame (parent → joint)
		glm::vec3 joint_origin_xyz{ 0.0f, 0.0f, 0.0f };
		glm::quat joint_origin_rpy{ 1.0, 0.0, 0.0, 0.0 };

		// Axis expressed IN JOINT FRAME
		glm::vec3 joint_axis{ 0.0f, 0.0f, 1.0f };

		// --- State ---
		float position = 0.0f;   // rad
		float omega = 0.0f;   // rad/s

		// --- Limits ---
		bool continuous = false;
		float minAngle = 0.0f; // rad
		float maxAngle = 0.0f; // rad
		float maxOmega = 1.0f; // rad/s

		// --- Control ---
		float targetPosition = 0.0f;
		float k_p = 25.0f;
		float k_d = 8.0f;
	};

	struct RobotModel {
		std::string name = "UnnamedRobot";	
		float scale = 1.0f; // metres per unit
		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;

		// Never used for scene hierarchy
		std::vector<kinematics::DH_Params> dhParams;

		// Create an Eigen vector of joint angles
		VecX makeJointVector() const {
			const int n = static_cast<int>(joints.size());
			VecX q(n);
			for (int i = 0; i < n; ++i) { q(i) = static_cast<double>(joints[i].angle); }
			return q;
		}

		// Set joint angles from an Eigen vector
		void setJointVector(const VecX& q) {
			const int n = static_cast<int>(joints.size());
			if (q.size() != n) {
				LOG_ERROR("Joint vector size mismatch: expected %d, got %d", n, q.size());
				D_ERROR("Joint vector size mismatch: expected %d, got %d", n, q.size());
				return;
			}
			for (int i = 0; i < n; ++i) {
				float a = static_cast<float>(q(i));
				joints[i].angle = a;
			}
		}
	};
}