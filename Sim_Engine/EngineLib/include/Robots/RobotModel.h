#pragma once
#pragma warning(disable : 4251)

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
	// --- Robot Model Kinematic Models ---
	enum class eKinematicsModel { URDF, DH };
	enum class eJointType { REVOLUTE, PRISMATIC };

	// --- Robot Model Links ---

	struct Inertia { float ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0; };

	struct Inertial {
		float mass = 0.0f;
		Vec3 com_xyz{ 0.0,0.0,0.0 };
		Inertia inertia{};
	};

	struct CollisionShape {
		std::string type;
		Vec3 size{ 0.0,0.0,0.0 }; // cylinder -> size = [radius, length, 0], box -> size = [x, y, z]

		Vec3 origin_xyz{ 0.0,0.0,0.0 };
		Vec3 origin_rpy{ 0.0,0.0,0.0 };

		std::string meshFile;	// Z1 provided STLs for collision meshes, dont use yet
	};

	struct Visual {
		std::string meshFile;
		Vec3 origin_xyz{ 0.0,0.0,0.0 };
		Vec3 origin_rpy{ 0.0,0.0,0.0 };
	};

	struct RobotLink {
		std::string name;
		Visual visual{};
		std::vector<CollisionShape> collisions;
		Inertial inertial{};

		scene::Object* attachedObject = nullptr;
	};

	// --- Robot Model Joints ---

	struct JointLimit {
		bool continuous = false;
		float minAngle = 0.0f;
		float maxAngle = 0.0f;
		float maxOmegaRad_s = glm::radians(180.0f);
		float maxEffort = 0.0f; // max torque/force
		// Soft limits
		double omegaRefMaxRad_s = 0.0;
	};

	struct JointDynamics {
		float damping = 0.0f;
		float friction = 0.0f;
	};

	struct RobotJoint {
		std::string name = "";
		std::string parent = "";
		std::string child = "";

		// URDF joint type
		eJointType type = eJointType::REVOLUTE;

		// NEW (in parent link local space)
		Vec3 axisParent = Vec3(0.0, 0.0, 1.0);
		Vec3 pivotParent = Vec3(0.0, 0.0, 0.0);

		// URDF joint frame (parent → joint)
		Vec3 origin_xyz{ 0.0, 0.0, 0.0 }; // translation from parent link frame to joint frame, expressed in parent link frame
		Vec3 origin_rpy{ 0.0, 0.0, 0.0 }; // roll, pitch, yaw in radians
		Quat origin_q{ 1,0,0,0 };         // Rotation matrix from link frame to base frame, derived from rpy_deg in JSON

		// Axis expressed IN JOINT FRAME
		Vec3 axis{ 0.0f, 0.0f, 1.0f };

		// --- Limits ---
		JointLimit limits;
		JointDynamics dynamics;

		// --- State ---
		float thetaRad = 0.0f;	 // rad
		float omegaRad_s = 0.0f; // rad/s
		float torque = 0.0f;	 // Nm or N
		float eta = 0.0f;		 // Integral state

		// --- Control ---
		float thetaRefRad = 0.0f;	 // rad
		float omegaRefRad_s = 0.0f;  // rad/s
		float alphaRefRad_s2 = 0.0f; // rad/s^2
		float k_p = 10.0f;	// position gain (rad)
		float k_i = 0.0f;	// integral gain (rad*s)
		float k_d = 10.0f;	// velocity gain (rad/s)

		float wn_target = 20.0f;   // rad/s
		float beta_target = 0.1f;  // overshoot ratio
		float zeta_target = 1.1f;  // damping ratio

		// --- Precomputed transforms ---
		Mat4 jointToChildRest = Mat4::Identity();
		Mat4 parentToJoint = Mat4::Identity();
	};

	// --- Robot Model ---

	struct RobotModel {
		std::string name = "UnnamedRobot";
		float scale = 1.0f;

		eKinematicsModel kinematicsModel = eKinematicsModel::URDF;
		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;

		std::vector<kinematics::DH_Params> dhParams;

		// Create an Eigen vector of joint angles
		VecX makeJointVector() const {
			const int n = static_cast<int>(joints.size());
			LOG_INFO_ONCE("Making joint vector of size %d", n);
			VecX q(n);
			for (int i = 0; i < n; ++i) { q(i) = static_cast<double>(joints[i].thetaRad); }
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
				joints[i].thetaRad = a;
			}
		}
	};

	// --- Robot Metrics ---

	// Per-joint metrics
	struct RobotMetrics {
		// Joint metrics
		double theta{ 0.0 }, omega{ 0.0 }, eta{ 0.0 };
		double thetaRef{ 0.0 }, omegaRef{ 0.0 }, alphaRef{ 0.0 };
		double err{ 0.0 }, err_d{ 0.0 };
		double I_eff{ 0.0 };
		double tau{ 0.0 }, tau_motor{ 0.0 }, tau_robot{ 0.0 }, tau_f{ 0.0 };
		double tau_barrier{ 0.0 }, tau_sat{ 0.0 };
		double wMax_hw{ 0.0 }, wMax_traj{ 0.0 };
		double traj_overspeed{ 0.0 };
		double c{ 0.0 }, mu{ 0.0 }, g{ 0.0 };
		double alpha{ 0.0 };
		double kp{ 0.0 }, kd{ 0.0 }, ki{ 0.0 };
		bool sat_flag, traj_overspeed_flag;
		// link metrics
		double mass{ 0.0 }, M_ii{ 0.0 };
		Mat3 I_link, I_world, R;
		Vec3 Jv, Jw;
		Vec3 com, v_com, omega_link;
	};

}