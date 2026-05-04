#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "core/constants.h"

using namespace mathlib;
using namespace constants;

namespace mathlib {
	// Converts a non-eigen vector to Vec3 (if it has 3 elements)
	template <typename T>
	inline Vec3 toVec3(const T& vec) {
		static_assert(std::tuple_size<T>::value == 3, "Input vector must have exactly 3 elements");
		return Vec3(vec[0], vec[1], vec[2]);
	}

	// Converts a non-eigen vector to Vec4 (if it has 4 elements)
	template <typename T>
	inline Vec4 toVec4(const T& vec) {
		static_assert(std::tuple_size<T>::value == 4, "Input vector must have exactly 4 elements");
		return Vec4(vec[0], vec[1], vec[2], vec[3]);
	}

	// Converts a non-eigen 3x3 matrix to Mat3 (if it has 3 rows and 3 columns)
	template <typename T>
	inline Mat3 toMat3(const T& mat) {
		static_assert(std::tuple_size<T>::value == 3 && std::tuple_size<typename T::value_type>::value == 3, "Input matrix must be 3x3");
		return Mat3(mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]
		);
	}

	// Converts a non-eigen 4x4 matrix to Mat4 (if it has 4 rows and 4 columns)
	template <typename T>
	inline Mat4 toMat4(const T& mat) {
		static_assert(std::tuple_size<T>::value == 4 && std::tuple_size<typename T::value_type>::value == 4, "Input matrix must be 4x4");
		return Mat4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]
		);
	}

	// Convert degrees to radians
	inline double deg2rad(double degrees) { return degrees * (PI_d / 180.0); }

	// Convert radians to degrees
	inline double rad2deg(double radians) { return radians * (180.0 / PI_d); }

	// Portable radian conversion
	inline double radians(double degrees) {
		return degrees * std::numbers::pi / 180.0;
	}
	// Portable degree conversion
	inline double degrees(double radians) {
		return radians * 180.0 / std::numbers::pi;
	}

	// Clamp a value between min and max
	inline double clamp(double value, double minVal, double maxVal) {
		if (value < minVal) { return minVal; }
		if (value > maxVal) { return maxVal; }
		return value;
	}

	// Method to wrap an angle in radians to the range [-pi, pi]
	inline double wrapToPi(double angleRad) {
		angleRad = std::fmod(angleRad + PI_d, TWO_PI_d);
		if (angleRad < 0.0f) { angleRad += TWO_PI_d; }
		return angleRad - PI_d; // [rad]
	}

	// Method to wrap an angle in radians to the range [0, 2pi]
	inline double wrapRad(double angleRad) {
		angleRad = fmod(angleRad, TWO_PI_d);
		if (angleRad < 0.0f) { angleRad += TWO_PI_d; }
		return angleRad; // [rad]
	}

	// Linear interpolation between a and b by factor t (0 <= t <= 1)
	inline double lerp(double a, double b, double t) { return a + t * (b - a); }

	// Check if two doubles are approximately equal within a tolerance 
	inline bool approximatelyEqual(double a, double b, double tolerance = 1e-9) { return std::fabs(a - b) <= tolerance; }

	// Check if a value is within a specified range [minVal, maxVal]
	inline bool isInRange(double value, double minVal, double maxVal) { return (value >= minVal) && (value <= maxVal); }

	// Sign function: returns -1 for negative, 1 for positive, and 0 for zero
	inline double sgn(double val) { return (val > 0) - (val < 0); }

	// Returns a quadratic function f(x) = a*x^2 + b*x + c
	inline std::function<double(double)> quadratic(double a, double b, double c) { return [a, b, c](double x) { return a * x * x + b * x + c; }; }

	// Returns a cubic function f(x) = a*x^3 + b*x^2 + c*x + d
	inline std::function<double(double)> cubic(double a, double b, double c, double d) { return [a, b, c, d](double x) { return a * x * x * x + b * x * x + c * x + d; }; }

	// Dot product of two Vec3
	inline double dot(const Vec3& v1, const Vec3& v2) { return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z(); }

	// Natural frequency of a mass-spring system
	inline double natural_freq(double k_p, double I) {
		if (!std::isfinite(k_p) || !std::isfinite(I)) return std::numeric_limits<double>::quiet_NaN();
		if (k_p < 0.0 || I <= 0.0) return std::numeric_limits<double>::quiet_NaN();
		return std::sqrt(k_p / I);
	}

	// Damping ratio of a mass-spring-damper system 
	inline double damping_ratio(double k_d, double I, double k_p) {
		if (!std::isfinite(k_p) || !std::isfinite(k_d) || !std::isfinite(I)) { return std::numeric_limits<double>::quiet_NaN(); }
		if (k_p <= 0.0 || I <= 0.0) { return std::numeric_limits<double>::quiet_NaN(); }
		double w_n = natural_freq(k_p, I);
		return k_d / (2.0 * I * w_n); // damping ratio - zeta
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

	// Overloads for std::array and C arrays for Vec4 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 4, Vec4> toVec4(const std::array<T, N>& arr) {
		return Vec4(arr[0], arr[1], arr[2], arr[3]);
	}
	template <typename T>
	inline Vec4 toVec4(const T arr[4]) {
		return Vec4(arr[0], arr[1], arr[2], arr[3]);
	}

	// Overloads for std::array and C arrays for Mat3 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 3, Mat3> toMat3(const std::array<std::array<T, N>, N>& mat) {
		return Mat3(mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]);
	}
	template <typename T>
	inline Mat3 toMat3(const T mat[3][3]) {
		return Mat3(mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]);
	}

	// Overloads for std::array and C arrays for Mat4 conversion
	template <typename T, std::size_t N>
	inline std::enable_if_t<N == 4, Mat4> toMat4(const std::array<std::array<T, N>, N>& mat) {
		return Mat4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
	}
	template <typename T>
	inline Mat4 toMat4(const T mat[4][4]) {
		return Mat4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
	}
}