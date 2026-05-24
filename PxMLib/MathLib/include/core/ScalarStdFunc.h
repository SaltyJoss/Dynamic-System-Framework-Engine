// PxM/MathLib ScalarStdFunc.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"
#include <cmath>

namespace mathlib {
	// Power function (Scalar)
	template<typename Scalar>
	inline Scalar pow(const Scalar& x, const Scalar& n) { return std::pow(x, n); }
	// Sqrt function (Scalar)
	template<typename Scalar>
	inline Scalar sqrt(const Scalar& x) { return std::sqrt(x); }
	// Sine function (Scalar)
	template<typename Scalar>
	inline Scalar sin(const Scalar& x) { return std::sin(x); }
	// Arcsine function (Scalar)
	template<typename Scalar>
	inline Scalar asin(const Scalar& x) { return std::asin(x); }
	// Cosine function (Scalar)
	template<typename Scalar>
	inline Scalar cos(const Scalar& x) { return std::cos(x); }
	// Arccosine function (Scalar)
	template<typename Scalar>
	inline Scalar acos(const Scalar& x) { return std::acos(x); }
	// Tangent function (Scalar)
	template<typename Scalar>
	inline Scalar tan(const Scalar& x) { return std::tan(x); }
	// Arctangent function (Scalar)
	template<typename Scalar>
	inline Scalar atan(const Scalar& x) { return std::atan(x); }
	// Arctangent sqaured function (Scalar)
	template<typename Scalar>
	inline Scalar atan2(const Scalar& y, const Scalar& x) { return std::atan2(y, x); }
	// Hyperbolic Tangnet (Scalar)
	template<typename Scalar>
	inline Scalar tanh(const Scalar& x) { return std::tanh(x); }
	// Exponential function (Scalar)
	template<typename Scalar>
	inline Scalar exp(const Scalar& x) { return std::exp(x); }
	// Logarithm function (Scalar)
	template<typename Scalar>
	inline Scalar log(const Scalar& x) { return std::log(x); }
	// Absolute value function (Scalar)
	template<typename Scalar>
	inline Scalar abs(const Scalar& a) { return (a < Scalar(0)) ? -a : a; }
	// Absolute value function squared (Scalar)
	template<typename Scalar>
	inline Scalar abs2(const Scalar& a) { return a * a; }
	// Absolute value with tolerance (Scalar, overload)
	template<typename Scalar>
	inline Scalar abs(const Scalar& a, const Scalar& tol) { return (abs(a) < tol) ? Scalar(0) : a; }
	// Signum function (Scalar)
	template<typename Scalar>
	inline Scalar sgn(const Scalar& a) { return (a > Scalar(0)) - (a < Scalar(0)); }
} // namespace mathlib