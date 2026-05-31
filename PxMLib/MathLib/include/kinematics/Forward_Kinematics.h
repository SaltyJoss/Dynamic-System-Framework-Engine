#pragma once

#include <core/MathLib.h>
#include "kinematics/DH_Params.h"
#include "kinematics/URDF_Types.h"
#include <cmath>
#include <cassert>
#include <vector>

using namespace mathlib;

namespace kinematics {
	class Forward_Kinematics {
	public:
		/// <summary>
		/// Forward Kinematics using Denavit-Hartenberg parameters
		/// </summary
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>End-effector pose as a 4x4 transformation matrix</returns>
		template<typename Scalar>
		inline Pose_T<Scalar> FK_DH(const std::vector<DH_Params<Scalar>>& dh_p, const VecX_T<Scalar>& q) {
			Pose_T<Scalar> T = Pose_T<Scalar>::Identity();   // Initialise as identity

			const size_t n = dh_p.size();  // number of joints
			assert(static_cast<std::size_t>(q.size()) >= n);   // basic safety

			for (std::size_t i = 0; i < n; ++i) {
				const DH_Params<Scalar>& p = dh_p[i];   // current joint parameters
				const Scalar joint = q(static_cast<Eigen::Index>(i));   // current joint variable
				Scalar theta = p.theta; // base angle
				Scalar d = p.d; // base offset

				if (p.type == JointType_DH::Revolute) { theta += joint; }
				else { d += joint; }

				const Scalar a = p.a;         // link length
				const Scalar alpha = p.alpha; // link twist

				const Scalar cth = mathlib::cos(theta);
				const Scalar sth = mathlib::sin(theta);
				const Scalar ca = mathlib::cos(alpha);
				const Scalar sa = mathlib::sin(alpha);

				Pose_T<Scalar> A; // individual link transform
				A << cth, -sth * ca, sth* sa, a* cth,			// row 1 - rotation
					sth, cth* ca, -cth * sa, a* sth,			// row 2 - rotation
					Scalar(0), sa, ca, d,						// row 3 - translation
					Scalar(0), Scalar(0), Scalar(0), Scalar(1);	// row 4 - homogeneous

				T = T * A; // accumulate
			}
			return T;  // end-effector pose
		}
		// Double overload
		inline Pose FK_DH(const std::vector<DH_Params<double>>& dh_p, const VecX& q) { return FK_DH<double>(dh_p, q); }

		/// <summary>
		/// Compute the transformation matrices for each link in the kinematic chain
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Vector of transformation matrices for each link</returns>
		template<typename Scalar>
		inline std::vector<Pose_T<Scalar>> linkTransforms_DH(const std::vector<DH_Params<Scalar>>& dh_p, const VecX_T<Scalar>& q, const Pose_T<Scalar>& T_base = Pose_T<Scalar>::Identity()) {
			std::vector<Pose_T<Scalar>> transforms;
			transforms.reserve(dh_p.size());   // avoid reallocs

			Pose_T<Scalar> T = T_base;   // Initialise as identity

			const std::size_t n = dh_p.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const DH_Params<Scalar>& p = dh_p[i];
				const Scalar joint = q(static_cast<Eigen::Index>(i));
				Scalar theta = p.theta;
				Scalar d = p.d;

				if (p.type == JointType_DH::Revolute) { theta += joint; }
				else { d += joint; }

				const Scalar a = p.a;
				const Scalar alpha = p.alpha;

				const Scalar cth = mathlib::cos(theta);
				const Scalar sth = mathlib::sin(theta);
				const Scalar ca = mathlib::cos(alpha);
				const Scalar sa = mathlib::sin(alpha);

				Pose_T<Scalar> A; // individual link transform
				A << cth, -sth * ca, sth* sa, a* cth,			// row 1 - rotation
					sth, cth* ca, -cth * sa, a* sth,			// row 2 - rotation
					Scalar(0), sa, ca, d,						// row 3 - translation
					Scalar(0), Scalar(0), Scalar(0), Scalar(1); // row 4 - homogeneous

				T = T * A;
				transforms.push_back(T);    // store current link transform
			}

			return transforms;
		}
		// Double overload
		inline std::vector<Pose> linkTransforms_DH(const std::vector<DH_Params<double>>& dh_p, const VecX& q, const Pose& T_base = Pose::Identity()) {
			return linkTransforms_DH<double>(dh_p, q, T_base);
		}

		/// <summary>
		/// Computes Forward Kinematics using URDF joint definitions
		/// </summary>
		/// <param name="joints"> A vector defining the robot's joints</param>
		/// <param name="q"> A vector of joint variables</param>
		/// <param name="T_base"> Optional base transformation matrix</param>
		/// <returns>End-effector pose as a 4x4 transformation matrix</returns>
		template<typename Scalar>
		inline Pose_T<Scalar> FK_URDF(const std::vector<JointURDF<Scalar>>& joints, const VecX_T<Scalar>& q, const Pose_T<Scalar>& T_base = Pose_T<Scalar>::Identity()) {
			Pose_T<Scalar> T = T_base;
			const size_t n = joints.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const auto& joint = joints[i];
				const Scalar qi = q(static_cast<Eigen::Index>(i)); // joint variable

				Pose_T<Scalar> T_origin = Pose_T<Scalar>::Identity(); // Joint origin transform
				T_origin.template block<3, 3>(0, 0) = joint.origin_R;
				T_origin.template block<3, 1>(0, 3) = joint.origin_xyz;

				// Axis in parent frame
				Vec3_T<Scalar> axis = safeNormalised(joint.axis);
				if (joint.axixInJointFrame) { axis = safeNormalised(joint.origin_R * axis); }

				// Compute joint motion transform
				Pose_T<Scalar> T_motion = Pose_T<Scalar>::Identity();
				if (joint.type == JointType_URDF::REVOLUTE) { T_motion.template block<3, 3>(0, 0) = AngleAxis<Scalar>(qi, axis); }
				else if (joint.type == JointType_URDF::PRISMATIC) { T_motion.template  block<3, 1>(0, 3) = axis * qi; }

				// Update total transform
				T = T * T_origin * T_motion;
			}
			return T;  // Placeholder implementation
		}
		// Double overload
		inline Pose FK_URDF(const std::vector<JointURDF<double>>& joints, const VecX& q, const Pose& T_base = Pose::Identity()) {
			return FK_URDF<double>(joints, q, T_base);
		}

		template<typename Scalar>
		std::vector<Pose_T<Scalar>> linkTransforms_URDF(const std::vector<JointURDF<Scalar>>& joints, const VecX_T<Scalar>& q, const Pose_T<Scalar>& T_base = Pose_T<Scalar>::Identity()) {
			std::vector<Pose_T<Scalar>> out;
			out.reserve(joints.size());

			Pose_T<Scalar> T = T_base;
			const std::size_t n = joints.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const auto& joint = joints[i];
				const Scalar qi = q(static_cast<Eigen::Index>(i)); // joint variable

				Pose_T<Scalar> T_origin = Pose_T<Scalar>::Identity(); // Joint origin transform
				T_origin.template block<3, 3>(0, 0) = joint.origin_R;
				T_origin.template block<3, 1>(0, 3) = joint.origin_xyz;

				// Axis in parent frame
				Vec3_T<Scalar> axis = safeNormalised(joint.axis);
				if (joint.axixInJointFrame) { axis = safeNormalised(joint.origin_R * axis); }

				// Compute joint motion transform
				Pose_T<Scalar> T_motion = Pose_T<Scalar>::Identity();
				if (joint.type == JointType_URDF::REVOLUTE) { T_motion.template block<3, 3>(0, 0) = AngleAxis<Scalar>(qi, axis); }
				else if (joint.type == JointType_URDF::PRISMATIC) { T_motion.template  block<3, 1>(0, 3) = axis * qi; }

				// Update total transform
				T = T * T_origin * T_motion;
				out.push_back(T);
			}
			return out;
		}
		// Double overload
		inline std::vector<Pose> linkTransforms_URDF(const std::vector<JointURDF<double>>& joints, const VecX& q, const Pose& T_base = Pose::Identity()) {
			return linkTransforms_URDF<double>(joints, q, T_base);
		}
	};
}