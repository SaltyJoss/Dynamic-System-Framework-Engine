#pragma once

#include "EngineCore.h"
#include <Robots/RobotModel.h>

// Types of selections in the simulation
enum class SelectionType {
	NONE,
	LINK,
	ROBOT
	// may add more types later (e.g., JOINT, SENSOR, OBJECT)
};

// Current selection state
struct Selection {
	SelectionType type = SelectionType::NONE;
	int index = -1; // Index of the selected link or robot
};


