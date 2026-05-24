// DSFE_Core RobotKinematics.h
#pragma once

#include "EngineCore.h"
#include "MathLibAPI.h"
#include <core/Types.h>
#include <core/Types_tpl.h>
#include <core/DualNumbers.h>
#include <core/Utils.h>
#include <kinematics/Forward_Kinematics.h>
#include "Robots/RobotSimSnapshot.h"

#include "EngineLib/LogMacros.h"

namespace robots {
	// Forward declarations
	struct RobotLink;
	struct RobotJoint;
	struct RobotMetrics;

	// Kinematics class responsible for computing forward kinematics and related transformations
	class DSFE_API RobotKinematics {
	public:
		// Constructor
		RobotKinematics();

		// Computes the forward kinematics for the robot based on the current state and robot configuration
		template<typename Scalar>
		void computeForwardKinematics_fromState(
			const RobotConstModel& robot,
			const mathlib::VecX_T<Scalar>& x,
			std::vector<mathlib::Pose_T<Scalar>>& T_world_out
		) const;

		// Computes the joint world poses for all joints based on the current state and robot configuration
		template<typename Scalar>
		std::vector<mathlib::Pose_T<Scalar>> calcJointWorldPoses(
			const std::vector<mathlib::Pose_T<Scalar>>& T_world,
			const RobotConstModel& robot
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
}

#include "Robots/RobotKinematics.inl"