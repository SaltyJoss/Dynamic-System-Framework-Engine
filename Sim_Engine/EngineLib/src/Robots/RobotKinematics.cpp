#include "pch.h"
// File:   RobotKinematics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotKinematics.h"
#include "Robots/RobotModel.h"
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotKinematics::RobotKinematics(RobotModel& robot)
		: _robot(robot) {
	}

	// Computes the forward kinematics for the robot based on the current state and robot configuration
	std::vector<mathlib::Pose> RobotKinematics::computeForwardKinematics_fromState(const mathlib::VecX& x) const {
		const auto& joints = _robot.joints;
		const auto& links = _robot.links;
		const size_t n = (size_t)joints.size();

		std::vector<Pose> T_world;
		T_world.reserve((size_t)links.size());

		Pose T = Pose::Identity(); // world -> base
		T_world.push_back(T);	   // base link 

		// Compute the transform to the next link using each joint
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = joints[i];
			const double theta = x[i]; // joint angle from state vector

			Pose T_origin = Pose::Identity(); // transform from parent link to joint frame (fixed)
			T_origin.block<3, 3>(0, 0) = joint.origin_q.toRotationMatrix(); // rotation from parent link frame to joint frame, derived from rpy in JSON
			T_origin.block<3, 1>(0, 3) = joint.origin_xyz;					// translation from parent link to joint frame

			// Compute joint motion transform based on joint axis and angle
			Pose T_motion = Pose::Identity();
			if (joint.type == eJointType::REVOLUTE) {
				T_motion = jointMotionTransform(joint.axis, theta); // rotation about joint axis
			}
			else if (joint.type == eJointType::PRISMATIC) {
				T_motion.block<3, 1>(0, 3) = joint.axis.normalized() * theta; // translation along joint axis
			}

			// compose transforms 
			T = T * T_origin * T_motion; // parent -> joint -> motion -> child
			T_world.push_back(T); // link i+1 pose in world frame
		}
		return T_world; // poses of all links in world frame
	}

	// Computes the joint world poses for all joints based on the current state and robot configuration
	std::vector<Pose> RobotKinematics::calcJointWorldPoses(
		const std::vector<Pose>& T_world,
		const std::vector<RobotJoint>& joints
	) {
		std::vector<Pose> jointWorldPoses;
		jointWorldPoses.reserve(joints.size());
		for (size_t i = 0; i < joints.size(); ++i) {
			const Pose& T = T_world[i + 1]; // joint i is at the end of link i, which is at T_world[i+1]
			jointWorldPoses.push_back(T);
		}
		return jointWorldPoses;
	}

	// Computes the forward kinematics for a single joint motion based on the joint axis and angle
	mathlib::Pose RobotKinematics::jointMotionTransform(
		const mathlib::Vec3& axis_joint,
		double theta
	) const {
		Pose T = Pose::Identity(); // homogeneous transformation matrix (4x4)

		Eigen::AngleAxisd aa(theta, axis_joint.normalized()); // create angle-axis rotation from joint angle and axis
		T.block<3, 3>(0, 0) = aa.toRotationMatrix();		  // set upper-left 3x3 block to rotation matrix

		return T; // (4x4) homogeneous transformation
	}

	// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
	mathlib::Quat RobotKinematics::rpyRadToQuat(const mathlib::Vec3& rpyRad) {
		const double roll = rpyRad.x();
		const double pitch = rpyRad.y();
		const double yaw = rpyRad.z();

		const Quat qx(Eigen::AngleAxisd(roll, Vec3(1.0, 0.0, 0.0)));
		const Quat qy(Eigen::AngleAxisd(pitch, Vec3(0.0, 1.0, 0.0)));
		const Quat qz(Eigen::AngleAxisd(yaw, Vec3(0.0, 0.0, 1.0)));

		return (qz * qy * qx).normalized();
	}

	// Accessor for the robot model
	void RobotKinematics::setRobot(RobotModel& robot) { _robot = robot; }
}