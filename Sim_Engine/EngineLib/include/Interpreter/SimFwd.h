// Header file to provide forward declarations of SimContext-related classes
#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "Platform/Logger.h"

namespace scene { 
	enum class ObjectID : std::uint32_t;
	class ENGINE_API Object;
}

namespace gui { class ENGINE_API simManager; }
namespace robots { class ENGINE_API RobotSystem; }
namespace physics { class ENGINE_API PhysicsSystem; }
