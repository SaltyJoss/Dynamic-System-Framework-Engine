#pragma once

#include "MathLibAPI.h"

namespace dynamics {
	struct MATHLIB_API LinkInertia {
		double mass;
		Vec3 com; // center of mass
		Mat3 inertia; // inertia tensor
	};
}