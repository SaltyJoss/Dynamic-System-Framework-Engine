#pragma once
// File:   RobotDynamics.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"

namespace robots {
	// Forward declarations
	struct ENGINE_API RobotModel;
	struct ENGINE_API RobotLink;
	struct ENGINE_API RobotJoint;
	struct ENGINE_API RobotMetrics;

	// Kinematics class responsible for computing forward kinematics and related transformations
	class ENGINE_API RobotKinematics {
	public:
		// Constructor
		RobotKinematics(RobotModel& robot);

		// Computes the forward kinematics for the robot based on the current state and robot configuration
		std::vector<mathlib::Pose> computeForwardKinematics_fromState(const mathlib::VecX& x) const;

		// Computes the joint world poses for all joints based on the current state and robot configuration
		std::vector<mathlib::Pose> calcJointWorldPoses(
			const std::vector<mathlib::Pose>& T_world,
			const std::vector<RobotJoint>& joints
		);

		// Computes the forward kinematics for a single joint motion based on the joint axis and angle
		mathlib::Pose jointMotionTransform(
			const mathlib::Vec3& axis_joint,
			double theta
		) const;

		// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
		mathlib::Quat rpyRadToQuat(const mathlib::Vec3& rpyRad);

		// Accessor for the robot model
		void setRobot(RobotModel& robot);

	private:
		RobotModel& _robot;
	};
}