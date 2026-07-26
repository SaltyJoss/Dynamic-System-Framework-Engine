/*
 * File: Physics/RigidBodyDynamics.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <MathLib>

#include "Physics/DynamicsTypes.h"
#include "Systems/RigidBodyMetrics.h"

#include "Physics/SpatialDynamics.h"
#include "Systems/SpatialModel.h"
#include "Physics/RigidBodyKinematics.h"
#include "Systems/RigidBodySimSnapshot.h"

#include "Systems/TrajectoryManager.h"

#include <cmath>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

// Forward declarations
namespace control { class TrajectoryManager; }
namespace integration { class IntegrationService; enum class eIntegrationMethod; }

namespace systems {
	// Forward declarations
	struct RigidBodyLink;
	struct RigidBodyJoint;
	enum class eTorqueMode;
} // namespace systems

namespace physics {
	// Dynamics class responsible for computing inertia, mass matrix, gravity torque, control torques, and state derivatives
	class DSFE_API RigidBodyDynamics {
	public:
		// Constructor
		RigidBodyDynamics();

		// Computes the inertia tensor of a body link
		template<typename Scalar>
		mathlib::Mat3_T<Scalar> computeLinkInertiaTensor(const Link& link) const;

		// Computes the contribution of a single joint and its child link to the effective inertia I_eff of the joint
		template<typename Scalar>
		Scalar computeJointInertiaContribution(
			const systems::RigidBodyJoint& joint,
			const systems::RigidBodyLink& link,
			const mathlib::Pose_T<Scalar>& jointWorldPose,
			const mathlib::Pose_T<Scalar>& linkWorldPose
		) const;

		// Computes the full mass matrix M(q) based on the current state and body configuration
		template<typename Scalar>
		void computeMassMatrix(
			const systems::RigidBodyConstModel& body,
			const std::vector<mathlib::Pose_T<Scalar>>& T_world,
			const std::vector<mathlib::Pose_T<Scalar>>& jointWorldPoses,
			mathlib::MatX_T<Scalar>& M_out
		) const;

		// Computes the Coriolis and centrifugal bias vector h(q, qd) based on the current state and body configuration
		template<typename Scalar>
		mathlib::VecX_T<Scalar> computeCoriolisVector(
			const systems::RigidBodyConstModel& body,
			const mathlib::VecX_T<Scalar>& q,
			const mathlib::VecX_T<Scalar>& qd,
			const std::vector<mathlib::Pose_T<Scalar>>& T_world,
			const mathlib::MatX_T<Scalar>& M
		) const;

		// Computes the gravity torque for a joint based on the current state and body configuration
		template<typename Scalar>
		mathlib::VecX_T<Scalar> computeGravityTorque(
			const systems::RigidBodyConstModel& body,
			const std::vector<mathlib::Pose_T<Scalar>>& T_world,
			const std::vector<mathlib::Pose_T<Scalar>>& jointWorldPoses
		) const;

		// Computes the analytical Jacobian matrix J(q) for the body based on the current state and body configuration
		template<typename Scalar>
		void analyticalJacobian(
			const systems::RigidBodyConstModel& body,
			const mathlib::VecX_T<Scalar>& x,
			mathlib::MatX_T<Scalar>& J_out,
			DenseDynamicsScratch<Scalar>& scratch
		);

		// Computes the Coriolis and centrifugal torque for a joint based on the current state and body configuration
		template<typename Scalar>
		mathlib::VecX_T<Scalar> derivative_dense(
			Scalar t,
			const mathlib::VecX_T<Scalar>& x,
			const systems::RigidBodySnapshot_T<Scalar>& snap,
			DynamicsScratch<Scalar>& scratch,
			DynamicsResult<Scalar>& out
		);

		template<typename Scalar>
		mathlib::VecX_T<Scalar> derivative_spatial(
			const systems::SpatialModel<Scalar>& model,
			Scalar t,
			const mathlib::VecX_T<Scalar>& x,
			const systems::RigidBodySnapshot_T<Scalar>& snap,
			DynamicsScratch<Scalar>& scratch,
			DynamicsResult<Scalar>& out
		);

		template<typename Scalar>
		void jacobian_spatial(
			const systems::SpatialModel<Scalar>& model,
			const mathlib::VecX_T<Scalar>& x,
			const systems::RigidBodySnapshot_T<Scalar>& snap,
			const mathlib::VecX_T<Scalar>& kp,
			const mathlib::VecX_T<Scalar>& kd,
			mathlib::MatX_T<Scalar>& F_out,
			DynamicsScratch<Scalar>& scratch
		);

		// Computes the derivative of the state vector with control gains based on the current state and body configurations
		template<typename Scalar>
		mathlib::VecX_T<Scalar> derivative_with_gains(
			Scalar t,
			const mathlib::VecX_T<Scalar>& x,
			const systems::RigidBodySnapshot_T<Scalar>& snap,
			const mathlib::VecX_T<Scalar>& kp,
			const mathlib::VecX_T<Scalar>& kd,
			DynamicsScratch<Scalar>& scratch,
			DynamicsResult<Scalar>& out
		);

		// Computes the Jacobian matrix with control gains based on the current state and body configuration
		template<typename Scalar>
		void jacobian_with_gains(
			const mathlib::VecX_T<Scalar>& x,
			const systems::RigidBodySnapshot_T<Scalar>& snap,
			const mathlib::VecX_T<Scalar>& kp,
			const mathlib::VecX_T<Scalar>& kd,
			mathlib::MatX_T<Scalar>& F_out,
			DenseDynamicsScratch<Scalar>& scratch
		);

		// Set the gravity strength for the body system
		void setGravity(double gravity) { _gravity = gravity; }
		const double getGravity() const { return _gravity; }

		// Set the timestep for dynamics updates (used for energy calculations and integration)
		void setDt(double dt) { _dt = dt; }
		const double dt() const { return _dt; }

	private:
		// References and pointers
		std::unique_ptr<RigidBodyKinematics> _kinematics = nullptr;

		static bool isControlledJoint(eJointType t) {
			return
				t == eJointType::REVOLUTE ||
				t == eJointType::PRISMATIC;
		}

		double _dt = 1.0 / 180.0; // default timestep for dynamics updates

		double _gravity{ 0.0 };
		bool _baseIsFree = false;
		double _lastBaseForwardForce{ 0.0 };
	};
} // namespace physics

#include "Systems/RigidBodyDynamics.inl"