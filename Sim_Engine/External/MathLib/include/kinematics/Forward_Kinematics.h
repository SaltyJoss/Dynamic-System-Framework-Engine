#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Params.h"
#include <cmath>
#include <cassert>

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
		Pose FK(const std::vector<DH_Params>& dh_p, const VecX& q) {
			Pose T = Pose::Identity();   // Initialize as identity

			const std::size_t n = dh_p.size();  // number of joints
			assert(static_cast<std::size_t>(q.size()) >= n);   // basic safety

			for (std::size_t i = 0; i < n; ++i) {
				const DH_Params& p = dh_p[i];   // current joint parameters
				const double joint = q(static_cast<Eigen::Index>(i));   // current joint variable
				double theta = p.theta; // base angle
				double d = p.d; // base offset

				if (p.type == JointType::Revolute) {
					theta += joint;   // q affects angle
				}
				else { // Prismatic
					d += joint;   // q affects offset
				}

				const double a = p.a;        // link length
				const double alpha = p.alpha;    // link twist

				const double cth = std::cos(theta);
				const double sth = std::sin(theta);
				const double ca = std::cos(alpha);
				const double sa = std::sin(alpha);

				Pose A; // individual link transform
				A << cth, -sth * ca,   sth* sa, a* cth, // row 1 - rotation
					 sth,	cth* ca, -cth * sa, a* sth, // row 2 - rotation
					 0.0,		 sa,		ca,		 d, // row 3 - translation
					 0.0,		0.0,       0.0,    1.0; // row 4 - homogeneous

				T = T * A; // accumulate

				//// Output Debug Info
				//std::cout << "FK Debug Info: "
				//	<< "i=" << i
				//	<< " a=" << a
				//	<< " d=" << d
				//	<< " theta=" << theta
				//	<< std::endl;
			}

			return T;  // end-effector pose
		}

		
		/// <summary>
		/// Compute the transformation matrices for each link in the kinematic chain
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Vector of transformation matrices for each link</returns>
		std::vector<Pose> linkTransforms(const std::vector<DH_Params>& dh_p, const VecX& q) { return linkTransforms(dh_p, q, Pose::Identity()); }

		/// <summary>
		/// Compute the transformation matrices for each link in the kinematic chain
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Vector of transformation matrices for each link</returns>
		std::vector<Pose> linkTransforms(const std::vector<DH_Params>& dh_p, const VecX& q, const Pose& T_base) {
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

				if (p.type == JointType::Revolute) {
					theta += joint;   // q affects angle
				}
				else { // Prismatic
					d += joint;   // q affects offset
				}

				const double a = p.a;
				const double alpha = p.alpha;

				const double cth = std::cos(theta);
				const double sth = std::sin(theta);
				const double ca = std::cos(alpha);
				const double sa = std::sin(alpha);

				Pose A; // individual link transform
				A << cth, -sth * ca,   sth* sa, a* cth, // row 1 - rotation
					 sth,	cth* ca, -cth * sa, a* sth, // row 2 - rotation
					 0.0,		 sa,		ca,		 d, // row 3 - translation
					 0.0,		0.0,       0.0,    1.0; // row 4 - homogeneous

				T = T * A;
				transforms.push_back(T);    // store current link transform

				// Debug info for each link
				std::string debugStr = "Link " + std::to_string(i) +
					": a=" + std::to_string(a) +
					", d=" + std::to_string(d) +
					", theta=" + std::to_string(theta);
			}

			return transforms;
		}
	};
}