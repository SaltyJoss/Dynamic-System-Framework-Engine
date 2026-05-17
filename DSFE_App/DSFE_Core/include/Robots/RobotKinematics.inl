// DSFE_Core RobotKinematics.inl
#pragma once

namespace robots {
	// Computes the forward kinematics for the robot based on the current state and robot configuration
	template<typename Scalar>
	void RobotKinematics::computeForwardKinematics_fromState(
		const RobotConstModel& robot,
		const mathlib::VecX_T<Scalar>& x,
		std::vector<mathlib::Pose_T<Scalar>>& T_world_out
	) const {
		const auto& joints = robot.joints;
		const auto& links = robot.links;
		const size_t n = joints.size();

		T_world_out.resize(robot.links.size());

		mathlib::Pose_T<Scalar> T = mathlib::Pose_T<Scalar>::Identity(); // world -> base
		T_world_out[0] = T;	   // base link

		if (T_world_out.empty()) {
			LOG_ERROR("T_world_out is empty");
			return;
		}

		// Compute the transform to the next link using each joint
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = joints[i];
			const Scalar q = x[i]; // joint angle from state vector

			mathlib::Pose_T<Scalar> T_origin = mathlib::Pose_T<Scalar>::Identity(); // transform from parent link to joint frame (fixed)
			T_origin.template block<3, 3>(0, 0) = joint.origin_q.toRotationMatrix(); // rotation from parent link frame to joint frame, derived from rpy in JSON
			T_origin.template block<3, 1>(0, 3) = joint.origin_xyz;					// translation from parent link to joint frame

			// Compute joint motion transform based on joint axis and angle
			Pose_T<Scalar> T_motion = mathlib::Pose_T<Scalar>::Identity();
			if (joint.type == eJointType::REVOLUTE) {
				T_motion = jointMotionTransform(joint.axis, q); // rotation about joint axis
			}
			else if (joint.type == eJointType::PRISMATIC) {
				T_motion.template block<3, 1>(0, 3) = joint.axis.normalized() * q; // translation along joint axis
			}

			// compose transforms 
			T = T * T_origin * T_motion; // parent -> joint -> motion -> child

			int childIdx = robot.linkIndex(joint.child);
			if (childIdx < 0 || childIdx >= T_world_out.size()) {
				LOG_ERROR("Invalid child link index for joint {}: {}", joint.name.c_str(), childIdx);
				continue;
			}

			T_world_out[childIdx] = T; // world -> child link
		}
	}

	// Computes the joint world poses for all joints based on the current state and robot configuration
	template<typename Scalar>
	std::vector<mathlib::Pose_T<Scalar>> RobotKinematics::calcJointWorldPoses(
		const std::vector<mathlib::Pose_T<Scalar>>& T_world,
		const RobotConstModel& robot
	) {
		std::vector<mathlib::Pose_T<Scalar>> jointWorldPoses(robot.joints.size());

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
	template<typename Scalar>
	mathlib::Pose_T<Scalar> RobotKinematics::jointMotionTransform(
		const mathlib::Vec3_T<Scalar>& axis_joint,
		Scalar q
	) const {
		mathlib::Pose_T<Scalar> T = mathlib::Pose_T<Scalar>::Identity(); // homogeneous transformation matrix (4x4)

		Eigen::AngleAxis<Scalar> aa(q, axis_joint.normalized()); // create angle-axis rotation from joint angle and axis
		T.template block<3, 3>(0, 0) = aa.toRotationMatrix();	 // set upper-left 3x3 block to rotation matrix

		return T; // (4x4) homogeneous transformation
	}

	// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
	template<typename Scalar>
	mathlib::Quat_T<Scalar> RobotKinematics::rpyRadToQuat(const mathlib::Vec3_T<Scalar>& rpyRad) {
		const double roll = rpyRad.x();
		const double pitch = rpyRad.y();
		const double yaw = rpyRad.z();

		const Quat_T<Scalar> qx(Eigen::AngleAxis<Scalar>(roll,	mathlib::Vec3_T<Scalar>(Scalar(1), Scalar(0), Scalar(0))));
		const Quat_T<Scalar> qy(Eigen::AngleAxis<Scalar>(pitch,	mathlib::Vec3_T<Scalar>(Scalar(0), Scalar(1), Scalar(0))));
		const Quat_T<Scalar> qz(Eigen::AngleAxis<Scalar>(yaw,	mathlib::Vec3_T<Scalar>(Scalar(0), Scalar(0), Scalar(1))));

		return (qz * qy * qx).normalized();
	}
}