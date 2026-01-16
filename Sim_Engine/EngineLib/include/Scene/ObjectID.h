#pragma once
#include "EngineCore.h"
#include <cstdint>

namespace scene {
	using ObjectID = std::uint32_t;
	constexpr ObjectID INVALID_OBJECT_ID = 0; // Reserved ID for invalid objects
}