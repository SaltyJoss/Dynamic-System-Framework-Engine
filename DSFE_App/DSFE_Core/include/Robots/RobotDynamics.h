// DSFE_Core RobotDynamics.h
#pragma once

#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"
#include "Robots/DynamicsTypes.h"
#include "Robots/RobotMetrics.h"

// Forward declarations
namespace control { class TrajectoryManager; }
namespace integration { class IntegrationService; enum class eIntegrationMethod; }

namespace robots {
	// Forward declarations
	class RobotKinematics;
	struct RobotConstModel;
	struct SpatialModel;
	struct RobotSimSnapshot;
	struct RobotLink;
	struct RobotJoint;
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
			const mathlib::Pose& jointWorldPose,
			const mathlib::Pose& linkWorldPose
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
			const mathlib::VecX& q, const mathlib::VecX& qd,
			const std::vector<mathlib::Pose>& T_world,
			const mathlib::MatX& M
		) const;

		// Computes the gravity torque for a joint based on the current state and robot configuration
		mathlib::VecX computeGravityTorque(
			const RobotConstModel& robot,
			const std::vector<mathlib::Pose>& T_world,
			const std::vector<mathlib::Pose>& jointWorldPoses
		) const;

		// Computes the analytical Jacobian matrix J(q) for the robot based on the current state and robot configuration
		void analyticalJacobian(
			const RobotConstModel& robot,
			const mathlib::VecX& x,
			mathlib::MatX& J_out,
			DenseDynamicsScratch& scratch
		);

		// Computes the Coriolis and centrifugal torque for a joint based on the current state and robot configuration
		mathlib::VecX derivative_dense(
			double t,
			const mathlib::VecX& x,
			const RobotSimSnapshot& snap,
			DynamicsScratch& scratch,
			DynamicsResult& out
		);

		mathlib::VecX derivative_spatial(
			const robots::SpatialModel& model,
			double t,
			const mathlib::VecX& x,
			const RobotSimSnapshot& snap,
			DynamicsScratch& scratch,
			DynamicsResult& out
		);

		void jacobian_spatial(
			const robots::SpatialModel& model,
			const mathlib::VecX& x,
			const RobotSimSnapshot& snap,
			const mathlib::VecX& kp,
			const mathlib::VecX& kd,
			mathlib::MatX& F_out,
			DynamicsScratch& scratch
		);

		// Computes the derivative of the state vector with control gains based on the current state and robot configurations
		mathlib::VecX derivative_with_gains(
			double t,
			const mathlib::VecX& x,
			const RobotSimSnapshot& snap,
			const mathlib::VecX& kp,const mathlib::VecX& kd,
			DynamicsScratch& scratch, DynamicsResult& out
		);

		// Computes the Jacobian matrix with control gains based on the current state and robot configuration
		void jacobian_with_gains(
			const mathlib::VecX& x, const RobotSimSnapshot& snap,
			const mathlib::VecX& kp, const mathlib::VecX& kd, mathlib::MatX& F_out,
			DenseDynamicsScratch& scratch
		);

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