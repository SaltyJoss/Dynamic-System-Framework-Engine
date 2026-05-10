#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace dynamics {
	struct MATHLIB_API LinkInertia {
		double mass;
		Vec3 com; // center of mass
		Mat3 inertia; // inertia tensor
	};
}