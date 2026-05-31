#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "core/ScalarStdFunc.h"
#include "core/ScalarScaling.h"
#include "core/constants.h"

using namespace mathlib;
using namespace constants;

namespace mathlib {
	// Convert degrees to radians (T)
	template<typename T>
	inline T radians(T degrees) { return degrees * (T(PI) / T(180)); }
	// Convert degrees to radians (double overload)
	inline double radians(double degrees) { return  radians<double>(degrees); }

	// Convert radians to degrees
	template<typename T>
	inline T degrees(T radians) { return radians * (T(180) / T(PI)); }
	// Convert radians to degrees (double overload)
	inline double degrees(double radians) { return degrees<double>(radians); }

	// Clamp a value between min and max
	template<typename T>
	inline T clamp(T value, T minVal, T maxVal) {
		if (value < minVal) { return minVal; }
		if (value > maxVal) { return maxVal; }
		return value;
	}
	// Clamp a value between min and max (double overload)
	inline double clamp(double value, double minVal, double maxVal) { return clamp<double>(value, minVal, maxVal); }

	// Method to wrap an angle in radians to the range [-pi, pi]
	template<typename T>
	inline T wrapToPi(T angleRad) {
		angleRad = mathlib::fmod(angleRad + T(PI), T(TWO_PI));
		if (angleRad < T(0)) { angleRad += T(TWO_PI); }
		return angleRad - T(PI); // [rad]
	}
	// Method to wrap an angle in radians to the range [-pi, pi] (double overload)
	inline double wrapToPi(double angleRad) { return wrapToPi<double>(angleRad); }

	// Method to wrap an angle in radians to the range [0, 2pi]
	template<typename T>
	inline T wrapRad(T angleRad) {
		angleRad = mathlib::fmod(angleRad, T(TWO_PI));
		if (angleRad < T(0)) { angleRad += T(TWO_PI); }
		return angleRad; // [rad]
	}
	// Method to wrap an angle in radians to the range [0, 2pi] (double overload)
	inline double wrapRad(double angleRad) { return wrapRad<double>(angleRad); }

	// Linear interpolation between a and b by factor t (0 <= t <= 1)
	inline double lerp(double a, double b, double t) { return a + t * (b - a); }

	// Check if two doubles are approximately equal within a tolerance
	inline bool approximatelyEqual(double a, double b, double tolerance = 1e-9) { return std::fabs(a - b) <= tolerance; }

	// Check if a value is within a specified range [minVal, maxVal]
	inline bool isInRange(double value, double minVal, double maxVal) { return (value >= minVal) && (value <= maxVal); }

	// Dot product of two 3D vectors
	template<typename T>
	inline T dot(const Vec3_T<T>& v1, const Vec3_T<T>& v2) { return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z(); }
	// Dot product of two 3D vectors (double overload)
	inline double dot(const Vec3& v1, const Vec3& v2) { return dot<double>(v1, v2); }
	// Cross product of two 3D vectors
	template<typename T>
	inline Vec3_T<T> cross(const Vec3_T<T>& v1, const Vec3_T<T>& v2) {
		return Vec3_T<T>(
			v1.y() * v2.z() - v1.z() * v2.y(),
			v1.z() * v2.x() - v1.x() * v2.z(),
			v1.x() * v2.y() - v1.y() * v2.x()
		);
	}
	// Cross product of two 3D vectors (double overload)
	inline Vec3 cross(const Vec3& v1, const Vec3& v2) { return cross<double>(v1, v2); }

	// Natural frequency of a mass-spring system
	template<typename T>
	inline T natural_freq(T k_p, T I) {
		if (!mathlib::isfinite(k_p) || !mathlib::isfinite(I)) { return std::numeric_limits<T>::quiet_NaN(); }
		if (k_p < T(0) || I <= T(0)) { return std::numeric_limits<T>::quiet_NaN(); }
		return mathlib::sqrt(k_p / I);
	}
	// Natural frequency of a mass-spring system (double overload)
	inline double natural_freq(double k_p, double I) { return natural_freq<double>(k_p, I); }

	// Damping ratio of a mass-spring-damper system 
	template<typename T>
	inline T damping_ratio(T k_d, T I, T k_p) {
		if (!mathlib::isfinite(k_p) || !mathlib::isfinite(k_d) || !mathlib::isfinite(I)) { return std::numeric_limits<T>::quiet_NaN(); }
		if (k_p < T(0) || I <= T(0)) { return std::numeric_limits<T>::quiet_NaN(); }
		T w_n = natural_freq(k_p, I);
		return k_d / (T(2) * I * w_n); // damping ratio - zeta
	}
	// Damping ratio of a mass-spring-damper system (double overload)
	inline double damping_ratio(double k_d, double I, double k_p) { return damping_ratio<double>(k_d, I, k_p); }

	// Converts a non-eigen vector to Vec3 (if it has 3 elements)
	template<typename T>
	inline Vec3 toVec3(const T& vec) {
		static_assert(std::tuple_size<T>::value == 3, "Input vector must have exactly 3 elements");
		return Vec3(vec[0], vec[1], vec[2]);
	}
	// Overloads for std::array and C arrays for Vec3 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 3, Vec3> toVec3(const std::array<T, N>& arr) {
		return Vec3(arr[0], arr[1], arr[2]);
	}
	template <typename T>
	inline Vec3 toVec3(const T arr[3]) {
		return Vec3(arr[0], arr[1], arr[2]);
	}

	// Converts a non-eigen vector to Vec4 (if it has 4 elements)
	template<typename T>
	inline Vec4 toVec4(const T& vec) {
		static_assert(std::tuple_size<T>::value == 4, "Input vector must have exactly 4 elements");
		return Vec4(vec[0], vec[1], vec[2], vec[3]);
	}
	// Overloads for std::array and C arrays for Vec4 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 4, Vec4> toVec4(const std::array<T, N>& arr) {
		return Vec4(arr[0], arr[1], arr[2], arr[3]);
	}
	template <typename T>
	inline Vec4 toVec4(const T arr[4]) {
		return Vec4(arr[0], arr[1], arr[2], arr[3]);
	}

	// Converts a non-eigen 3x3 matrix to Mat3 (if it has 3 rows and 3 columns)
	template<typename T>
	inline Mat3 toMat3(const T& mat) {
		static_assert(std::tuple_size<T>::value == 3 && std::tuple_size<typename T::value_type>::value == 3, "Input matrix must be 3x3");
		return Mat3(
			mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]
		);
	}
	// Overloads for std::array and C arrays for Mat3 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 3, Mat3> toMat3(const std::array<std::array<T, N>, N>& mat) {
		return Mat3(
			mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]
		);
	}
	template <typename T>
	inline Mat3 toMat3(const T mat[3][3]) {
		return Mat3(
			mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]
		);
	}

	// Converts a non-eigen 4x4 matrix to Mat4 (if it has 4 rows and 4 columns)
	template <typename T>
	inline Mat4 toMat4(const T& mat) {
		static_assert(std::tuple_size<T>::value == 4 && std::tuple_size<typename T::value_type>::value == 4, "Input matrix must be 4x4");
		return Mat4(
			mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]
		);
	}
	// Overloads for std::array and C arrays for Mat4 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 4, Mat4> toMat4(const std::array<std::array<T, N>, N>& mat) {
		return Mat4(
			mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]
		);
	}
	template <typename T>
	inline Mat4 toMat4(const T mat[4][4]) {
		return Mat4(
			mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]
		);
	}
}