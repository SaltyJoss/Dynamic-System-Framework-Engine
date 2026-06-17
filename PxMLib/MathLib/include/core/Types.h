#pragma once

#include "Types_tpl.h"

namespace mathlib {
	// Basic type definitions
	using uint = unsigned int;
	using uchar = unsigned char;
	using ushort = unsigned short;
	using ulong = unsigned long;
	using ullong = unsigned long long;

	// Double and float Eigen types using the template aliases defined in Types_tpl.h
	using Vec2 = Vec2_T<double>;
	using Vec2f = Vec2_T<float>;
	using Vec3 = Vec3_T<double>;
	using Vec3f = Vec3_T<float>;
	using Vec4 = Vec4_T<double>;
	using Vec4f = Vec4_T<float>;
	using VecX = VecX_T<double>;
	using VecXf = VecX_T<float>;
	using Mat2 = Mat2_T<double>;
	using Mat2f = Mat2_T<float>;
	using Mat3 = Mat3_T<double>;
	using Mat3f = Mat3_T<float>;
	using Mat4 = Mat4_T<double>;
	using Mat4f = Mat4_T<float>;
	using MatX = MatX_T<double>;
	using MatXf = MatX_T<float>;
	using Pose = Pose_T<double>;
	using Posef = Pose_T<float>;
	using Quat = Quat_T<double>;
	using Quatf = Quat_T<float>;

}