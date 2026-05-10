// DSFE_Core RobotKinematics.h
#pragma once

#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"

namespace robots {
	// Forward declarations
	struct RobotConstModel;
	struct RobotSimSnapshot;
	struct RobotLink;
	struct RobotJoint;
	struct RobotMetrics;

	// Kinematics class responsible for computing forward kinematics and related transformations
	class DSFE_API RobotKinematics {
	public:
		// Constructor
		RobotKinematics();

		// Computes the forward kinematics for the robot based on the current state and robot configuration
		void computeForwardKinematics_fromState(
			const RobotConstModel& robot,
			const mathlib::VecX& x,
			std::vector<mathlib::Pose>& T_world_out
		) const;

		// Computes the joint world poses for all joints based on the current state and robot configuration
		std::vector<mathlib::Pose> calcJointWorldPoses(
			const std::vector<mathlib::Pose>& T_world,
			const std::vector<RobotJoint>& joints
		);

		// Computes the forward kinematics for a single joint motion based on the joint axis and angle
		mathlib::Pose jointMotionTransform(
			const mathlib::Vec3& axis_joint,
			double q
		) const;

		// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
		mathlib::Quat rpyRadToQuat(const mathlib::Vec3& rpyRad);
	};
}