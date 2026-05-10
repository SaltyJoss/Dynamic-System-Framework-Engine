#include "pch.h"
// File:   RobotKinematics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotKinematics.h"
#include "Robots/RobotSimSnapshot.h"
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotKinematics::RobotKinematics() {
	}

	// Computes the forward kinematics for the robot based on the current state and robot configuration
	void RobotKinematics::computeForwardKinematics_fromState(
		const RobotConstModel& robot,
		const mathlib::VecX& x,
		std::vector<mathlib::Pose>& T_world_out
	) const {
		const auto& joints = robot.joints;
		const auto& links = robot.links;
		const size_t n = (size_t)joints.size();

		T_world_out.resize(robot.links.size());

		Pose T = Pose::Identity(); // world -> base
		T_world_out[0] = T;	   // base link 

		// Compute the transform to the next link using each joint
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = joints[i];
			const double q = x[i]; // joint angle from state vector

			Pose T_origin = Pose::Identity(); // transform from parent link to joint frame (fixed)
			T_origin.block<3, 3>(0, 0) = joint.origin_q.toRotationMatrix(); // rotation from parent link frame to joint frame, derived from rpy in JSON
			T_origin.block<3, 1>(0, 3) = joint.origin_xyz;					// translation from parent link to joint frame

			// Compute joint motion transform based on joint axis and angle
			Pose T_motion = Pose::Identity();
			if (joint.type == eJointType::REVOLUTE) {
				T_motion = jointMotionTransform(joint.axis, q); // rotation about joint axis
			}
			else if (joint.type == eJointType::PRISMATIC) {
				T_motion.block<3, 1>(0, 3) = joint.axis.normalized() * q; // translation along joint axis
			}

			// compose transforms 
			T = T * T_origin * T_motion; // parent -> joint -> motion -> child
			T_world_out[i + 1] = T;
		}
	}

	// Computes the joint world poses for all joints based on the current state and robot configuration
	std::vector<Pose> RobotKinematics::calcJointWorldPoses(
		const std::vector<Pose>& T_world,
		const RobotConstModel& robot
	) {
		std::vector<Pose> jointWorldPoses(robot.joints.size());

		for (size_t i = 0; i < robot.joints.size(); ++i) {
			const RobotJoint& joints = robot.joints[i];
			int childIdx = robot.linkIndex(joints.child);

			if (childIdx < 0 || childIdx >= T_world.size()) {
				LOG_ERROR("Invalid child link index for joint {}: {}", joints.name.c_str(), childIdx);
				continue;
			}

			jointWorldPoses[i] = T_world[childIdx];
		}
		return jointWorldPoses;
	}

	// Computes the forward kinematics for a single joint motion based on the joint axis and angle
	mathlib::Pose RobotKinematics::jointMotionTransform(
		const mathlib::Vec3& axis_joint,
		double q
	) const {
		Pose T = Pose::Identity(); // homogeneous transformation matrix (4x4)

		Eigen::AngleAxisd aa(q, axis_joint.normalized()); // create angle-axis rotation from joint angle and axis
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
}