#include "pch.h"
// File:   RobotKinematics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotKinematics.h"
#include "Robots/RobotSimSnapshot.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotKinematics::RobotKinematics() {
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