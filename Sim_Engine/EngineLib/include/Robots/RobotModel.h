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

		glm::mat4 meshFix = glm::mat4(1.0f); // visual correction
	};

	struct RobotJoint {
		std::string name = "";
		std::string parent = "";
		std::string child = "";

		glm::vec3 axis{ 0.0, 0.0, 0.0 };
		glm::vec3 offset{ 0.0, 0.0, 0.0 };
		glm::quat quat{ 1.0, 0.0, 0.0, 0.0 }; // initial orientation

		bool continuous = false; // true for base, false for limited joints
		float minAngle = 0.0f; // lower limit 
		float maxAngle = 0.0f; // upper limit 

		float angle = 0.0f; // current angle (rad)
		float omega = 0.0f; // angular velocity (rad/s)

		float thetaRef = 0.0f; // reference angle (rad)
		float maxOmega = 1.0f; // max |omega| (rad/s)

		float k_p = 25.0f; // position gain (rad/s^2)
		float k_d = 8.0f;  // velocity gain (rad/s^2)
	};

	struct RobotModel {
		std::string name = "UnnamedRobot";	
		float scale = 1.0f; // metres per unit
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