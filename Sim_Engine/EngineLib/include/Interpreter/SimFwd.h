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
	class DSFE_API Object;
}
namespace core { struct DSFE_API ISimulationCore; }
namespace robots { class DSFE_API RobotSystem; }
namespace physics { class DSFE_API PhysicsSystem; }
