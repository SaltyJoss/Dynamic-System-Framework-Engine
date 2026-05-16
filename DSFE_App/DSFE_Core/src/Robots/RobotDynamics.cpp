#include "pch.h"
// File:   RobotDynamics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotDynamics.h"
#include "Robots/RobotKinematics.h"
#include "Robots/RobotSimSnapshot.h"

#include "Robots/TrajectoryManager.h"

#include <cmath>

#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

namespace robots {
	// Constructor
	RobotDynamics::RobotDynamics() 
		: _kinematics(std::make_unique<RobotKinematics>()) {
	}
}