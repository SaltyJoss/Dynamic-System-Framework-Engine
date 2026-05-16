#include "pch.h"
// File:   RobotDynamics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotDynamics.h"

namespace robots {
	// Constructor
	RobotDynamics::RobotDynamics() 
		: _kinematics(std::make_unique<RobotKinematics>()) {
	}
}