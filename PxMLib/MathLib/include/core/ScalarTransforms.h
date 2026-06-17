// PxM/MathLib ScalarTransforms.h
#pragma once

#include "core/Types_tpl.h"
#include "core/SpatialMath.h"

namespace mathlib {
	template<typename Scalar>
	inline Mat3_T<Scalar> AngleAxis(const Scalar& theta, const Vec3_T<Scalar>& axis) {
		Vec3_T<Scalar> axis_n = safeNormalised(axis);
		Mat3_T<Scalar> K = skewSymmetric(axis_n);
		return Mat3_T<Scalar>::Identity() + sin(theta) * K + (Scalar(1) - cos(theta)) * K * K;
	}
}