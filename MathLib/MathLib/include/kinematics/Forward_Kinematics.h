#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
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
		Pose FK_DH(const std::vector<DH_Params>& dh_p, const VecX& q) {
			Pose T = Pose::Identity();   // Initialize as identity

			const std::size_t n = dh_p.size();  // number of joints
			assert(static_cast<std::size_t>(q.size()) >= n);   // basic safety

			for (std::size_t i = 0; i < n; ++i) {
				const DH_Params& p = dh_p[i];   // current joint parameters
				const double joint = q(static_cast<Eigen::Index>(i));   // current joint variable
				double theta = p.theta; // base angle
				double d = p.d; // base offset

				if (p.type == JointType_DH::Revolute) { theta += joint; }
				else { d += joint; }

				const double a = p.a;        // link length
				const double alpha = p.alpha;    // link twist

				const double cth = std::cos(theta);
				const double sth = std::sin(theta);
				const double ca = std::cos(alpha);
				const double sa = std::sin(alpha);

				Pose A; // individual link transform
				A << cth, -sth * ca, sth* sa, a* cth, // row 1 - rotation
					sth, cth* ca, -cth * sa, a* sth, // row 2 - rotation
					0.0, sa, ca, d, // row 3 - translation
					0.0, 0.0, 0.0, 1.0; // row 4 - homogeneous

				T = T * A; // accumulate
			}

			return T;  // end-effector pose
		}

		/// <summary>
		/// Compute the transformation matrices for each link in the kinematic chain
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Vector of transformation matrices for each link</returns>
		std::vector<Pose> linkTransforms_DH(const std::vector<DH_Params>& dh_p, const VecX& q, const Pose& T_base = Pose::Identity()) {
			std::vector<Pose> transforms;
			transforms.reserve(dh_p.size());   // avoid reallocs

			Pose T = T_base;   // Initialize as identity

			const std::size_t n = dh_p.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const DH_Params& p = dh_p[i];
				const double joint = q(static_cast<Eigen::Index>(i));
				double theta = p.theta;
				double d = p.d;

				if (p.type == JointType_DH::Revolute) { theta += joint; }
				else { d += joint; }

				const double a = p.a;
				const double alpha = p.alpha;

				const double cth = std::cos(theta);
				const double sth = std::sin(theta);
				const double ca = std::cos(alpha);
				const double sa = std::sin(alpha);

				Pose A; // individual link transform
				A << cth, -sth * ca, sth* sa, a* cth, // row 1 - rotation
					sth, cth* ca, -cth * sa, a* sth, // row 2 - rotation
					0.0, sa, ca, d, // row 3 - translation
					0.0, 0.0, 0.0, 1.0; // row 4 - homogeneous

				T = T * A;
				transforms.push_back(T);    // store current link transform
			}

			return transforms;
		}

		/// <summary>
		/// Computes Forward Kinematics using URDF joint definitions
		/// </summary>
		/// <param name="joints"> A vector defining the robot's joints</param>
		/// <param name="q"> A vector of joint variables</param>
		/// <param name="T_base"> Optional base transformation matrix</param>
		/// <returns>End-effector pose as a 4x4 transformation matrix</returns>
		Pose FK_URDF(const std::vector<JointURDF>& joints, const VecX& q, const Pose& T_base = Pose::Identity()) {
			Pose T = T_base;
			const std::size_t n = joints.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const auto& joint = joints[i];
				const double qi = q(static_cast<Eigen::Index>(i)); // joint variable

				Pose T_origin = Pose::Identity(); // Joint origin transform
				T_origin.block<3, 3>(0, 0) = joint.origin_R;
				T_origin.block<3, 1>(0, 3) = joint.origin_xyz;

				// Axis in parent frame
				Vec3 axis = joint.axis.normalized();
				if (joint.axixInJointFrame) { axis = (joint.origin_R * axis).normalized(); } // Transform axis to joint frame if needed

				// Compute joint motion transform
				Pose T_motion = Pose::Identity();
				if (joint.type == JointType_URDF::REVOLUTE) {
					Eigen::AngleAxisd aa(qi, axis);
					T_motion.block<3, 3>(0, 0) = aa.toRotationMatrix();
				}
				else if (joint.type == JointType_URDF::PRISMATIC) {
					T_motion.block<3, 1>(0, 3) = axis * qi;
				}

				// Update total transform
				T = T * T_origin * T_motion;
			}
			return T;  // Placeholder implementation
		}


		std::vector<Pose> linkTransforms_URDF(const std::vector<JointURDF>& joints, const VecX& q, const Pose& T_base = Pose::Identity()) {
			std::vector<Pose> out;
			out.reserve(joints.size());

			Pose T = T_base;
			const std::size_t n = joints.size();
			assert(static_cast<std::size_t>(q.size()) >= n);

			for (std::size_t i = 0; i < n; ++i) {
				const auto& joint = joints[i];
				const double qi = q(static_cast<Eigen::Index>(i)); // joint variable

				Pose T_origin = Pose::Identity(); // Joint origin transform
				T_origin.block<3, 3>(0, 0) = joint.origin_R;
				T_origin.block<3, 1>(0, 3) = joint.origin_xyz;

				// Axis in parent frame
				Vec3 axis = joint.axis.normalized();
				if (joint.axixInJointFrame) { axis = (joint.origin_R * axis).normalized(); } // Transform axis to joint frame if needed

				// Compute joint motion transform
				Pose T_motion = Pose::Identity();
				if (joint.type == JointType_URDF::REVOLUTE) {
					Eigen::AngleAxisd aa(qi, axis);
					T_motion.block<3, 3>(0, 0) = aa.toRotationMatrix();
				}
				else if (joint.type == JointType_URDF::PRISMATIC) {
					T_motion.block<3, 1>(0, 3) = axis * qi;
				}

				// Update total transform
				T = T * T_origin * T_motion;
				out.push_back(T);
			}
			return out;
		}
	};
}