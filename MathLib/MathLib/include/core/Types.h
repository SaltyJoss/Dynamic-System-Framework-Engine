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

	using Vec6 = Eigen::Matrix<double, 6, 1>;	// A 6D vector, often used to represent spatial velocities (linear and angular) or twists in robotics and kinematics
	using Vec7 = Eigen::Matrix<double, 7, 1>;	// A 7D vector, commonly used to represent a pose in 3D space (position and orientation) using a combination of translation (3D) and rotation (4D quaternion)

	using Pose = Eigen::Matrix4d;		// A 4x4 transformation matrix that combines rotation and translation and represents the pose of an object in 3D space
	using Quat = Eigen::Quaterniond;	// A quaternion representing rotation in 3D space, composed of one real part and three imaginary parts (x, y, z, w)
	using Quatf = Eigen::Quaternionf;	// A quaternion with single-precision floating-point coefficients, used for representing rotations in 3D space with less memory usage than double-precision quaternions

}