#pragma once

#include "MathLibAPI.h"

namespace mathlib {
	// Convert std::array to Eigen::Vector3d
	Vec3 toVec3(const std::array<double, 3>& arr) {
		return Vec3(arr[0], arr[1], arr[2]);
	}

	// Convert degrees to radians
	double deg2rad(double degrees) {
		return degrees * (M_PI / 180.0);
	}

	// Convert radians to degrees
	double rad2deg(double radians) {
		return radians * (180.0 / M_PI);
	}

	// Compute the Euclidean norm of a vector
	double norm(const VecX& v) {
		return v.norm();
	}

	// Compute the squared Euclidean norm of a vector
	double normSquared(const VecX& v) {
		return v.squaredNorm();
	}

	// Clamp a value between min and max
	double clamp(double value, double minVal, double maxVal) {
		if (value < minVal) return minVal;
		if (value > maxVal) return maxVal;
		return value;
	}

	// Linear interpolation between a and b by factor t (0 <= t <= 1)
	double lerp(double a, double b, double t) {
		return a + t * (b - a);
	}

	// Check if two doubles are approximately equal within a tolerance
	bool approximatelyEqual(double a, double b, double tolerance = 1e-9) {
		return std::fabs(a - b) <= tolerance;
	}

	// Check if a value is within a specified range [minVal, maxVal]
	bool isInRange(double value, double minVal, double maxVal) {
		return (value >= minVal) && (value <= maxVal);
	}
}