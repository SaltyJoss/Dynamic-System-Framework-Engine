#pragma once
// File:   RobotModel.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <Kinematics/DH_Params.h>
#include "Scene/Object.h"

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

namespace robots {
	// --- Robot Model Kinematic Models ---
	enum class eKinematicsModel {
		URDF,
		DH
	};
	/// --- URDF Joint Types ---
	enum class eJointType {
		FIXED = 0,
		REVOLUTE = 1,
		PRISMATIC = 2
	};
	/// --- Visual Frame Options ---
	enum class eVisualFrame {
		NONE,
		JOINT,
		LINK,
		WORLD
	};

	// --- Robot Model Links ---

	// Inertia tensor struct, representing the inertia of a link about its center of mass, expressed in the link's local frame
	struct Inertia { double ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0; };

	// Inertial properties of a link
	struct Inertial {
		double mass = 0.0f;
		Vec3 com_xyz{ 0.0,0.0,0.0 };
		Inertia inertia{};
	};

	// Collision shape struct, supporting basic shapes (box, cylinder) and mesh (not implemented yet)
	struct CollisionShape {
		std::string type;

		// Collision Geometry
		Vec3 origin_xyz{ 0.0, 0.0, 0.0 };
		Vec3 origin_rpy{ 0.0, 0.0, 0.0 };

		// Collision Geometry Parameters
		Vec3 size{ 0.0, 0.0, 0.0 }; // cylinder -> size = [radius, length, 0], box -> size = [x, y, z]
		std::string meshFile; // for mesh collision shapes, not implemented yet
		Vec4 material{ 0.7f, 0.0f, 0.2f, 1.0f };
		float metallic = 0.5f;
		float roughness = 0.5f;
	};

	// Per-mesh entry with individual material properties
	struct VisualMeshEntry {
		std::string meshFile;
		Vec4 material{ 0.7, 0.0, 0.2, 1.0 };
		float metallic = 0.5f;
		float roughness = 0.5f;
		bool hasMaterial = false; // true if material was explicitly specified
	};

	// Visual struct, representing the visual geometry of a link
	struct Visual {
		// Visual Geometry
		Vec3 origin_xyz{ 0.0, 0.0, 0.0 };
		Vec3 origin_rpy{ 0.0, 0.0, 0.0 };

		// Visual Geometry Parameters
		std::string meshFile;
		std::vector<std::string> meshFiles; // for multiple visual meshes per link (legacy, string-only)
		std::vector<VisualMeshEntry> meshEntries; // for multiple visual meshes with per-mesh material
		Vec4 material{ 0.5, 0.5, 0.5, 1.0 }; // default grey material if not specified at mesh level
		float metallic = 0.5f;
		float roughness = 0.5f;
	};

	// RobotLink struct, representing a single link in the robot model
	struct RobotLink {
		std::string name;

		// Geometries (for rendering)
		Visual visual{};
		std::vector<CollisionShape> collisions;
		Inertial inertial{};

		// Attached scene objects (one per visual mesh part)
		std::vector<scene::Object*> attachedObjects;
		scene::Object* attachedObject = nullptr; // primary object (first in attachedObjects)
	};

	// --- Robot Model Joints ---

	// Joint limits struct, representing the physical limits of a joint
	struct JointLimit {
		bool continuous = false;
		double minAngle = 0.0f;
		double maxAngle = 0.0f;
		double maxOmegaRad_s = glm::radians(180.0f);
		double maxEffort = 0.0f; // max torque/force
		// Soft limits
		double omegaRefMaxRad_s = 0.0;
	};

	// Joint dynamics parameters, representing the damping and friction properties of a joint
	struct JointDynamics {
		double damping = 0.0f;
		double friction = 0.0f;
	};

	// RobotJoint struct, representing a single joint in the robot model
	struct RobotJoint {
		// Joint name and parent-child link names
		std::string name = "";
		std::string parent = "";
		std::string child = "";

		// URDF joint type
		eJointType type = eJointType::REVOLUTE;

		// Parent joint axis and pivot (for visualization of the joint frame)
		Vec3 axisParent{ 0.0, 0.0, 1.0 };
		Vec3 pivotParent{ 0.0, 0.0, 0.0 };

		// URDF joint frame (parent → joint)
		Vec3 origin_xyz{ 0.0, 0.0, 0.0 }; // translation from parent link frame to joint frame, expressed in parent link frame
		Vec3 origin_rpy{ 0.0, 0.0, 0.0 }; // roll, pitch, yaw in radians
		Quat origin_q{ 1,0,0,0 };         // Rotation matrix from link frame to base frame, derived from rpy_deg in JSON

		// Axis expressed IN JOINT FRAME
		Vec3 axis{ 0.0, 0.0, 1.0 };

		// --- Limits ---
		JointLimit limits;
		JointDynamics dynamics;

		// --- State ---
		double thetaRad = 0.0f;	 // rad
		double omegaRad_s = 0.0f; // rad/s
		double torque = 0.0f;	 // Nm or N
		double eta = 0.0f;		 // Integral state

		// --- Control ---
		double thetaRefRad = 0.0f;	 // rad
		double omegaRefRad_s = 0.0f;  // rad/s
		double alphaRefRad_s2 = 0.0f; // rad/s^2
		double k_p = 10.0f;	// position gain (rad)
		double k_i = 0.0f;	// integral gain (rad*s)
		double k_d = 10.0f;	// velocity gain (rad/s)

		double wn_target = 20.0f;   // rad/s
		double beta_target = 0.1f;  // overshoot ratio
		double zeta_target = 1.1f;  // damping ratio

		// --- Precomputed transforms ---
		Mat4 jointToChildRest = Mat4::Identity();
		Mat4 parentToJoint = Mat4::Identity();
	};

	// --- Robot Model ---

	// RobotModel struct, representing the entire robot model
	struct RobotModel {
		std::string name = "UnnamedRobot";
		float scale = 1.0f;

		// Links and joints
		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;

		// Kinematics model (URDF or DH)
		eKinematicsModel kinematicsModel = eKinematicsModel::URDF;
		std::vector<kinematics::DH_Params> dhParams;

		// Visualization options
		eVisualFrame visualFrame = eVisualFrame::JOINT;
		std::unordered_map<std::string, Vec4> materials;
		Mat4 baseFrame = Mat4::Identity(); // transform from world frame to robot base frame, can be set in JSON

		bool baseFrameIsEngineAligned = false;

		// Create an Eigen vector of joint angles
		VecX makeJointVector() const {
			const int n = static_cast<int>(joints.size());
			LOG_INFO_ONCE("Making joint vector of size %d", n);
			VecX q(n);
			for (int i = 0; i < n; ++i) { q(i) = joints[i].thetaRad; }
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
				double a = q(i);
				joints[i].thetaRad = a;
			}
		}
	};

	// --- Robot Metrics ---

	// Per-joint metrics
	struct RobotMetrics {
		// State
		double theta{ 0.0 };
		double omega{ 0.0 };
		double alpha{ 0.0 };
		double err{ 0.0 };
		double err_d{ 0.0 };

		// Dynamics
		double I_eff{ 0.0 };
		double tau{ 0.0 };
		double tau_fb{ 0.0 };
		double tau_damping{ 0.0 };
		double tau_friction{ 0.0 };
		double tau_coriolis{ 0.0 };
		double tau_gravity{ 0.0 };

		// Constraints / realism
		double tau_barrier{ 0.0 };
		double tau_sat{ 0.0 };
		double wMax_hw{ 0.0 };
		double wMax_traj{ 0.0 };
		double traj_overspeed{ 0.0 };

		// Stability flags
		bool sat_flag{ false };
		bool traj_overspeed_flag{ false };
	};
} // namespace robots