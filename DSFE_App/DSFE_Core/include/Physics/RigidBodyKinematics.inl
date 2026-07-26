/*
 * File: Physics/RigidBodyKinematics.inl
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

namespace physics {
	// Computes the forward kinematics for the body based on the current state and body configuration
	template<typename Scalar>
	void RigidBodyKinematics::computeForwardKinematics_fromState(
		const systems::RigidBodyConstModel& body,
		const mathlib::VecX_T<Scalar>& x,
		std::vector<mathlib::Pose_T<Scalar>>& T_world_out
	) const {
		const auto& joints = body.joints;
		const auto& links = body.links;
		const size_t n = joints.size();

		T_world_out.resize(body.links.size());

		if (T_world_out.empty()) {
			LOG_ERROR("T_world_out is empty");
			return;
		}

		mathlib::Pose_T<Scalar> T = mathlib::Pose_T<Scalar>::Identity(); // world -> base
		T_world_out[0] = T;	   // base link

		// Compute the transform to the next link using each joint
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = joints[i];
			const Scalar q = x[i]; // joint angle from state vector

			mathlib::Pose_T<Scalar> T_origin = mathlib::Pose_T<Scalar>::Identity(); // transform from parent link to joint frame (fixed)
			mathlib::Quat_T<Scalar> q_origin = joint.origin_q.template cast<Scalar>(); // convert quaternion to correct scalar type
			
			T_origin.template block<3, 3>(0, 0) = q_origin.toRotationMatrix(); // rotation from parent link frame to joint frame, derived from rpy in JSON
			T_origin.template block<3, 1>(0, 3) = joint.origin_xyz.template cast<Scalar>(); // translation from parent link to joint frame

			// Compute joint motion transform based on joint axis and angle
			Pose_T<Scalar> T_motion = mathlib::Pose_T<Scalar>::Identity();
			if (joint.type == systems::eJointType::REVOLUTE) {
				T_motion = jointMotionTransform<Scalar>(joint.axis.template cast<Scalar>(), q); // rotation about joint axis
			}
			else if (joint.type == systems::eJointType::PRISMATIC) {
				T_motion.template block<3, 1>(0, 3) = mathlib::safeNormalised(joint.axis) * q; // translation along joint axis
			}

			// compose transforms 
			T = T * T_origin * T_motion; // parent -> joint -> motion -> child

			int childIdx = body.linkIndex(joint.child);
			if (childIdx < 0 || childIdx >= T_world_out.size()) {
				LOG_ERROR("Invalid child link index for joint {}: {}", joint.name.c_str(), childIdx);
				continue;
			}

			T_world_out[childIdx] = T; // world -> child link
		}
	}

	// Computes the joint world poses for all joints based on the current state and body configuration
	template<typename Scalar>
	std::vector<mathlib::Pose_T<Scalar>> RigidBodyKinematics::calcJointWorldPoses(
		const std::vector<mathlib::Pose_T<Scalar>>& T_world,
		const systems::RigidBodyConstModel& body
	) {
		std::vector<mathlib::Pose_T<Scalar>> jointWorldPoses(body.joints.size());

		for (size_t i = 0; i < body.joints.size(); ++i) {
			const systems::RigidBodyJoint& joints = body.joints[i];
			int childIdx = body.linkIndex(joints.child);

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
	mathlib::Pose_T<Scalar> RigidBodyKinematics::jointMotionTransform(
		const mathlib::Vec3_T<Scalar>& axis_joint,
		Scalar q
	) const {
		mathlib::Pose_T<Scalar> T = mathlib::Pose_T<Scalar>::Identity(); // homogeneous transformation matrix (4x4)
		T.template block<3, 3>(0, 0) = mathlib::AngleAxis(q, mathlib::safeNormalised(axis_joint)); // set upper-left 3x3 block to rotation matrix
		return T; // (4x4) homogeneous transformation
	}

	// Converts roll-pitch-yaw angles (in radians) to a quaternion representation
	template<typename Scalar>
	mathlib::Quat_T<Scalar> RigidBodyKinematics::rpyRadToQuat(const mathlib::Vec3_T<Scalar>& rpyRad) {
		const Scalar roll = rpyRad.x();
		const Scalar pitch = rpyRad.y();
		const Scalar yaw = rpyRad.z();

		const Quat_T<Scalar> qx(Eigen::AngleAxis<Scalar>(roll, mathlib::Vec3_T<Scalar>(Scalar(1), Scalar(0), Scalar(0))));
		const Quat_T<Scalar> qy(Eigen::AngleAxis<Scalar>(pitch, mathlib::Vec3_T<Scalar>(Scalar(0), Scalar(1), Scalar(0))));
		const Quat_T<Scalar> qz(Eigen::AngleAxis<Scalar>(yaw, mathlib::Vec3_T<Scalar>(Scalar(0), Scalar(0), Scalar(1))));

		return (qz * qy * qx).normalized();
	}
}