#include "pch.h"
#include "kinematics/Forward_Kinematics.h"

#include <cmath>        // for std::sin, std::cos
#include <cassert>      // optional, for assert

namespace kinematics {

    /// <inheritdoc/>
    Pose Forward_Kinematics::FK(const std::vector<DH_Params>& dh_p, const VecX& q) {
        Pose T = Pose::Identity();   // Initialize as identity

        const std::size_t n = dh_p.size();
        assert(static_cast<std::size_t>(q.size()) >= n);   // basic safety

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

            const double a = p.a;        // link length
            const double alpha = p.alpha;    // link twist

            const double cth = std::cos(theta);
            const double sth = std::sin(theta);
            const double ca = std::cos(alpha);
            const double sa = std::sin(alpha);

            Pose A; // current joint transform
            A << cth, -sth * ca, sth* sa, a* cth,
                sth, cth* ca, -cth * sa, a* sth,
                0.0, sa, ca, d,
                0.0, 0.0, 0.0, 1.0;

            T = T * A; // accumulate
        }

        return T;  // end-effector pose
    }

    /// <inheritdoc/>
    std::vector<Pose> Forward_Kinematics::linkTransformas(const std::vector<DH_Params>& dh_p, const VecX& q) {
        std::vector<Pose> transforms;
        transforms.reserve(dh_p.size());   // avoid reallocs

        Pose T = Pose::Identity();

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

            Pose A;
            A << cth, -sth * ca, sth* sa, a* cth,
                sth, cth* ca, -cth * sa, a* sth,
                0.0, sa, ca, d,
                0.0, 0.0, 0.0, 1.0;

            T = T * A;
            transforms.push_back(T);    // store current link transform
        }

        return transforms;
    }

}