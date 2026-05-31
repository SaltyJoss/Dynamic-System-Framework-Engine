// PxM/MathLib ScalarScaling.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"

namespace mathlib {
	// Smooth step function for smooth interpolation between 0 and 1
	template<typename Scalar>
	inline Scalar smoothStep(const Scalar& x) { return x * x * (Scalar(3) - Scalar(2) * x); }
	// LogSumExp Smooth Max (Scalar)
	template<typename Scalar>
	inline Scalar LSE_smoothMax(const Scalar& a, const Scalar& b, const Scalar& k = Scalar(10)) {
		Scalar m = (a > b) ? a : b;
		return m + mathlib::log(mathlib::exp(k * (a - m)) + mathlib::exp(k * (b - m))) / k;
	}

	// Maximum value function (Scalar)
	template<typename Scalar>
	inline Scalar max(const Scalar& a, const Scalar& b) { return (a > b) ? a : b; }
	// Minimum value function (Scalar)
	template<typename Scalar>
	inline Scalar min(const Scalar& a, const Scalar& b) { return (a < b) ? a : b; }

	// Check if a scalar is finite -> could use std::isfinite, but this avoids potential issues with non-standard types
	inline bool isfinite(double x) { return std::isfinite(x); }
	inline bool isfinite(float x) { return std::isfinite(x); }
}