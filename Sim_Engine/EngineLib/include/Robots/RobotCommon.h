#pragma once
// File:   RobotCommon.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "MathLibAPI.h"
#include "core/Types.h"

namespace robots {
	// Dynamics Mode
	enum class eTorqueMode {
		NONE,		// No physics simulation, just kinematics (e.g., for testing)
		PASSIVE,	// Physics simulation with passive joints (e.g., for observing natural dynamics or testing underactuated behavior)
		CONTROLLED	// Full physics simulation with active control (e.g., for testing control algorithms, trajectory tracking, or simulating real-world behavior)
	};
}