// PxM/MathLib ScalarScaling.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"

namespace mathlib {
	// LogSumExp Smooth Max (Scalar)
	template<typename Scalar>
	inline Scalar LSE_smoothMax(const Scalar& a, const Scalar& b, const Scalar& k = Scalar(10)) {
		Scalar m = (a > b) ? a : b;
		return m + log(exp(k * (a - m)) + exp(k * (b - m))) / k;
	}

	// Maximum value function (Scalar)
	template<typename Scalar>
	inline Scalar max(const Scalar& a, const Scalar& b) { return (a > b) ? a : b; }
	// Minimum value function (Scalar)
	template<typename Scalar>
	inline Scalar min(const Scalar& a, const Scalar& b) { return (a < b) ? a : b; }

	// Is finite function (Scalar)
	template<typename Scalar>
	inline bool isfinite(const Scalar& a) { return std::isfinite(a); }

	template<typename Derived>
	inline auto safeNorm(const Eigen::MatrixBase<Derived>& v) {
		using Scalar = typename Derived::Scalar;
		return mathlib::sqrt(v.dot(v));
	}
	template<typename Derived>
	inline typename Derived::PlainObject safeNormalised(const Eigen::MatrixBase<Derived>& v) {
		using Scalar = typename Derived::Scalar;
		using Plain = typename Derived::PlainObject;
		Scalar n = safeNorm(v);
		if (n > Scalar(0)) { return (v / n).eval(); }
		return Plain::Zero(v.rows(), v.cols());
	}
}