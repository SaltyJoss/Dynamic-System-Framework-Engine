#pragma once
// File:    SimFwd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "Platform/Logger.h"

// Forward declarations for the main classes used in the interpreter
namespace scene { 
	enum class ObjectID : std::uint32_t;
	class ENGINE_API Object;
}
namespace gui { class ENGINE_API SimManager; }
namespace robots { class ENGINE_API RobotSystem; }
namespace physics { class ENGINE_API PhysicsSystem; }
