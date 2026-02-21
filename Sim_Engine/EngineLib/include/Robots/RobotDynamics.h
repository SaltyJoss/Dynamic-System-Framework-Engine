#pragma once
// File:   RobotDynamics.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"

// Forward declarations
namespace control { class ENGINE_API TrajectoryManager; }
namespace integration { class ENGINE_API IntegrationService; enum class eIntegrationMethod; }

namespace robots {
	// Forward declarations
	class ENGINE_API RobotKinematics;
	struct ENGINE_API RobotModel;
	struct ENGINE_API RobotLink;
	struct ENGINE_API RobotJoint;
	struct ENGINE_API RobotMetrics;
	enum class eTorqueMode;

	// Dynamics class responsible for computing inertia, mass matrix, gravity torque, control torques, and state derivatives
	class ENGINE_API RobotDynamics {
	public:
		// Constructor
		RobotDynamics(RobotModel& robot, eTorqueMode mode);

		// Computes the inertia tensor of a robot link
		mathlib::Mat3 computeLinkInertiaTensor(const RobotLink& link) const;

		// Computes the contribution of a single joint and its child link to the effective inertia I_eff of the joint
		double computeJointInertiaContribution(
			const RobotJoint& joint,
			const RobotLink& link,
			const mathlib::Pose& jointWorldPose,   // pose of joint frame in world
			const mathlib::Pose& linkWorldPose     // pose of the link in world
		) const;

		// Computes the full mass matrix M(q) based on the current state and robot configuration
		mathlib::MatX computeMassMatrix(
			const std::vector<double>& q,
			const std::vector<mathlib::Pose>& T_world
		) const;

		// Computes the gravity torque for a joint based on the current state and robot configuration
		std::vector<double> computeGravityTorque(
			const std::vector<double>& q,
			const std::vector<mathlib::Pose>& T_world,
			mathlib::VecX x
		) const;

		// Computes the control torque for a joint based on the current state, reference, and robot configuration
		mathlib::VecX computeAppliedTorques(
			const std::vector<double>& q,
			const std::vector<double>& qd,
			const std::vector<double>& eta,
			const std::vector<mathlib::Pose>& T_world,
			std::vector<double> I_eff,
			std::vector<double> tau_gravity
		) const;

		// Computes control and dynamics metrics for a specific joint based on the current state and reference
		RobotMetrics computeJointMetrics(
			const RobotJoint& joint, const RobotLink& link, 
			double I_eff,
			double q, double qd, double eta,
			double q_ref, double qd_ref, double qdd_ref,
			double tau_coriolis, double tau_g
		) const;

		// Computes the Coriolis and centrifugal torque for a joint based on the current state and robot configuration
		mathlib::VecX derivative(
			double t,
			const mathlib::VecX& x
		) const;

		// Set the torque mode for the robot system
		void setTorqueMode(eTorqueMode mode) { _torqueMode = mode; }
		eTorqueMode getTorqueMode() const { return _torqueMode; }

		// Accessor for the robot model
		void setRobot(RobotModel& robot);

		// Set the gravity strength for the robot system
		void setGravity(double gravity) { _gravity = gravity; }

	private:
		// References and pointers
		RobotModel& _robot;
		std::unique_ptr<RobotKinematics> _kinematics;

		eTorqueMode _torqueMode;
		double _gravity{ 0.0 };
		bool _baseIsFree = false;
		double _lastBaseForwardForce{ 0.0 };
	};
} // namespace robots
