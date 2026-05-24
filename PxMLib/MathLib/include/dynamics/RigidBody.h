#pragma once

#include <core/MathLib.h>

using namespace mathlib;

namespace dynamics {
	template<typename Scalar>
	struct LinkInertia {
		Scalar mass;
		Vec3_T<Scalar> com; // center of mass
		Mat3_T<Scalar> inertia; // inertia tensor
	};
}