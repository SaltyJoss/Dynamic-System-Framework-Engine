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
	// --- Robot Model Links ---

	struct Inertia { float ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0; };

	struct Inertial {
		float mass = 0.0f;
		glm::vec3 com_xyz{ 0,0,0 };
		Inertia inertia{};
	};

	struct CollisionShape {
		std::string type;
		glm::vec3 origin_xyz{ 0,0,0 };
		glm::vec3 origin_rpy{ 0,0,0 };

		// cylinder: size = [radius, length, 0]
		// box:      size = [x, y, z]
		glm::vec3 size{ 0,0,0 };
		std::string meshFile;
	};

	struct Visual {
		std::string meshFile;
		glm::vec3 origin_xyz{ 0,0,0 };
		glm::vec3 origin_rpy{ 0,0,0 };
	};

	struct RobotLink {
		std::string name;
		Visual visual{};
		std::vector<CollisionShape> collisions;
		Inertial inertial{};

		// render-only correction (optional)
		glm::quat dhToMeshFix{ 1,0,0,0 };

		scene::Object* attachedObject = nullptr;
	};

	// --- Robot Model Joints ---

	struct JointLimit {
		bool continuous = false;
		float minAngle = 0.0f;
		float maxAngle = 0.0f;
		float maxOmegaRad_s = glm::radians(180.0f);
	};

	struct JointDynamics {
		float damping = 0.0f;
		float friction = 0.0f;
	};

	struct RobotJoint {
		std::string name = "";
		std::string parent = "";
		std::string child = "";

		// URDF joint frame (parent → joint)
		glm::vec3 origin_xyz{ 0.0f, 0.0f, 0.0f };
		glm::quat origin_q{ 1,0,0,0 }; // derived from rpy_deg in JSON

		// Axis expressed IN JOINT FRAME
		glm::vec3 axis{ 0.0f, 0.0f, 1.0f };

		// --- Limits ---
		JointLimit limits;
		JointDynamics dynamics;

		// --- State ---
		float angleRad = 0.0f;	// rad
		float omegaRad_s = 0.0f;// rad/s

		// --- Control ---
		float thetaRefRad = 0.0f;
		float k_p = 25.0f;
		float k_d = 8.0f;
	};

	// --- Robot Model ---

	struct RobotModel {
		std::string name = "UnnamedRobot";
		float scale = 1.0f;
		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;

		// Optional DH for kinematics ONLY
		std::vector<kinematics::DH_Params> dhParams;

		// Create an Eigen vector of joint angles
		VecX makeJointVector() const {
			const int n = static_cast<int>(joints.size());
			VecX q(n);
			for (int i = 0; i < n; ++i) { q(i) = static_cast<double>(joints[i].angleRad); }
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
				joints[i].angleRad = a;
			}
		}
	};
}