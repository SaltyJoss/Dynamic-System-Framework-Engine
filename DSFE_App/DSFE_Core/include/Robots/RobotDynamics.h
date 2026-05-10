// DSFE_Core RobotDynamics.h
#pragma once

#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"

// Forward declarations
namespace control { class TrajectoryManager; }
namespace integration { class IntegrationService; enum class eIntegrationMethod; }

namespace robots {
	// Forward declarations
	class RobotKinematics;
	struct RobotConstModel;
	struct RobotSimSnapshot;
	struct RobotLink;
	struct RobotJoint;
	struct RobotMetrics;
	enum class eTorqueMode;

	// Dynamics class responsible for computing inertia, mass matrix, gravity torque, control torques, and state derivatives
	class DSFE_API RobotDynamics {
	public:
		// Constructor
		RobotDynamics();

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
		void computeMassMatrix(
			const RobotConstModel& robot,
			const std::vector<mathlib::Pose>& T_world,
			const std::vector<mathlib::Pose>& jointWorldPoses,
			mathlib::MatX& M_out
		) const;

		// Computes the Coriolis and centrifugal bias vector h(q, qd) based on the current state and robot configuration
		mathlib::VecX computeCoriolisVector(
			const RobotConstModel& robot,
			const std::vector<double>& q,
			const std::vector<double>& qd,
			const std::vector<mathlib::Pose>& T_world,
			const mathlib::MatX& M
		) const;

		// Computes the gravity torque for a joint based on the current state and robot configuration
		std::vector<double> computeGravityTorque(
			const RobotConstModel& robot,
			const std::vector<mathlib::Pose>& T_world,
			const std::vector<mathlib::Pose>& jointWorldPoses
		) const;

		// Computes control and dynamics metrics for a specific joint based on the current state and reference
		RobotMetrics computeJointMetrics(
			const RobotSimSnapshot& snap,
			const RobotJoint& joint, double I_eff,
			double q, double qd,
			double q_ref, double qd_ref, double qdd_ref,
			double tau_coriolis, double tau_g
		) const;

		// Computes the Coriolis and centrifugal torque for a joint based on the current state and robot configuration
		mathlib::VecX derivative(
			double t,
			const mathlib::VecX& x,
			const RobotSimSnapshot& snap
		) const;

		// Set the gravity strength for the robot system
		void setGravity(double gravity) { _gravity = gravity; }
		const double getGravity() const { return _gravity; }

		// Set the timestep for dynamics updates (used for energy calculations and integration)
		void setDt(double dt) { _dt = dt; }
		const double dt() const { return _dt; }

	private:
		// References and pointers
		std::unique_ptr<RobotKinematics> _kinematics = nullptr;

		double _dt = 1.0 / 180.0; // default timestep for dynamics updates

		double _gravity{ 0.0 };
		bool _baseIsFree = false;
		double _lastBaseForwardForce{ 0.0 };
	};
} // namespace robots