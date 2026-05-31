// PxM/MathLib VectorUtils.h
#pragma once

#include "core/ScalarScaling.h"
#include "core/DualNumbers.h"
#include <Eigen/Dense>

namespace mathlib {
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