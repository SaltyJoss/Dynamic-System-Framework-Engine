// DSFE_Core RigidBodyKinematics.h
#pragma once

#include "EngineCore.h"
#include <MathLib>
#include <kinematics/Forward_Kinematics.h>
#include "Systems/RigidBodySnapshot.h"

#include "EngineLib/LogMacros.h"

// Forward declarations
namespace systems {
	struct RigidBodyLink;
	struct RigidBodyJoint;
}

namespace physics {
	// Kinematics class responsible for computing forward kinematics and related transformations
	class DSFE_API RigidBodyKinematics {
	public:
		// Constructor
		RigidBodyKinematics();

		// Computes the forward kinematics for the body based on the current state and body configuration
		template<typename Scalar>
		void computeForwardKinematics_fromState(
			const systems::RigidBodyConstModel& body,
			const mathlib::VecX_T<Scalar>& x,
			std::vector<mathlib::Pose_T<Scalar>>& T_world_out
		) const;

		// Computes the joint world poses for all joints based on the current state and body configuration
		template<typename Scalar>
		std::vector<mathlib::Pose_T<Scalar>> calcJointWorldPoses(
			const std::vector<mathlib::Pose_T<Scalar>>& T_world,
			const systems::RigidBodyConstModel& body
		);

		// Computes the forward kinematics for a single joint motion based on the joint axis and angle
		template<typename Scalar>
		mathlib::Pose_T<Scalar> jointMotionTransform(
			const mathlib::Vec3_T<Scalar>& axis_joint,
			Scalar q
		) const;

		// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
		template<typename Scalar>
		mathlib::Quat_T<Scalar> rpyRadToQuat(const mathlib::Vec3_T<Scalar>& rpyRad);
	};
} // namespace physics

#include "Physics/RigidBodyKinematics.inl"