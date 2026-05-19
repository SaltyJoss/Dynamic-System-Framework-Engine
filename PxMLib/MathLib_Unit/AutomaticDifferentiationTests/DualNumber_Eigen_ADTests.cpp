#include "TestHarness.h"
#include <core/DualNumbers.h>

using namespace mathlib;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;
}