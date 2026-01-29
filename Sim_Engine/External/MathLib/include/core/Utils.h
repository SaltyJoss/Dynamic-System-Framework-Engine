#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "core/constants.h"

using namespace mathlib;
using namespace constants;

namespace mathlib {
	// Convert std::array to Eigen::Vector3d
	Vec3 toVec3(const std::array<double, 3>& arr) { return Vec3(arr[0], arr[1], arr[2]); }

	// Convert degrees to radians
	double deg2rad(double degrees) { return degrees * (PI / 180.0); }

	// Convert radians to degrees
	double rad2deg(double radians) { return radians * (180.0 / PI); }

	// Clamp a value between min and max
	double clamp(double value, double minVal, double maxVal) {
		if (value < minVal) return minVal;
		if (value > maxVal) return maxVal;
		return value;
	}

	// Linear interpolation between a and b by factor t (0 <= t <= 1)
	double lerp(double a, double b, double t) { return a + t * (b - a); }

	// Check if two doubles are approximately equal within a tolerance 
	bool approximatelyEqual(double a, double b, double tolerance = 1e-9) { return std::fabs(a - b) <= tolerance; }

	// Check if a value is within a specified range [minVal, maxVal]
	bool isInRange(double value, double minVal, double maxVal) { return (value >= minVal) && (value <= maxVal); }

	// Sign function: returns -1 for negative, 1 for positive, and 0 for zero
	double sgn(double val) { return (val > 0) - (val < 0); }

	// Returns a quadratic function f(x) = a*x^2 + b*x + c
	std::function<double(double)> quadratic(double a, double b, double c) { return [a, b, c](double x) { return a * x * x + b * x + c; }; }

	// Returns a cubic function f(x) = a*x^3 + b*x^2 + c*x + d
	std::function<double(double)> cubic(double a, double b, double c, double d) { return [a, b, c, d](double x) { return a * x * x * x + b * x * x + c * x + d; }; }

	// Dot product of two Vec3
	double dot(const Vec3& v1, const Vec3& v2) { return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z(); }

	// Natural frequency of a mass-spring system
	double natural_freq(double k_p, double I) {
		if (!std::isfinite(k_p) || !std::isfinite(I)) return std::numeric_limits<double>::quiet_NaN();
		if (k_p < 0.0 || I <= 0.0) return std::numeric_limits<double>::quiet_NaN();
		return std::sqrt(k_p / I);
	}

	// Damping ratio of a mass-spring-damper system 
	double damping_ratio(double k_d, double I, double k_p) {
		if (!std::isfinite(k_p) || !std::isfinite(k_d) || !std::isfinite(I)) { return std::numeric_limits<double>::quiet_NaN(); }
		if (k_p <= 0.0 || I <= 0.0) { return std::numeric_limits<double>::quiet_NaN(); }
		double w_n = natural_freq(k_p, I);
		return k_d / (2.0 * I * w_n); // damping ratio - zeta
	}
}