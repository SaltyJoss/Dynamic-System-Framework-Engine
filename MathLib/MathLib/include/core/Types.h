#pragma message("Generating Types.h")
#pragma once

#include "MathLibAPI.h"
#include <Eigen/Dense>

namespace mathlib {
	// Basic type definitions
	using uint = unsigned int;
	using uchar = unsigned char;
	using ushort = unsigned short;
	using ulong = unsigned long;
	using ullong = unsigned long long;

	// Eigen type aliases
	using Vec3 = Eigen::Vector3d;
	using Vec4 = Eigen::Vector4d;
	using VecX = Eigen::VectorXd;
	using Mat3 = Eigen::Matrix3d;
	using Mat4 = Eigen::Matrix4d;
	using MatX = Eigen::MatrixXd;

	using Pose = Eigen::Matrix4d;		// A 4x4 transformation matrix that combines rotation and translation and represents the pose of an object in 3D space
	using Quat = Eigen::Quaterniond;	// A quaternion representing rotation in 3D space, composed of one real part and three imaginary parts (x, y, z, w)

}