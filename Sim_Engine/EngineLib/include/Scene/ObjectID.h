#pragma once
// File:   ObjectID.h
// GitHub: SaltyJoss
#include <cstdint>

// ObjectID is a strongly-typed identifier for scene objects, using uint32_t as the underlying type.
namespace scene {
	enum class ObjectID : std::uint32_t { INVALID_OBJECT_ID = 0 };				// Special invalid ObjectID
	inline constexpr ObjectID FIRST_VALID_OBJECT_ID = static_cast<ObjectID>(1); // First valid ObjectID

	inline std::uint32_t toUInt32(ObjectID id) { return static_cast<std::uint32_t>(id); }	 // Helper to convert ObjectID to uint32_t
	inline ObjectID fromUInt32(std::uint32_t value) { return static_cast<ObjectID>(value); } // Helper to convert uint32_t to ObjectID
	inline ObjectID nextID(ObjectID id) { return fromUInt32(toUInt32(id) + 1u); }			 // Helper to get the next ObjectID
}// namespace scene