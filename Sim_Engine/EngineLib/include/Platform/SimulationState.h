#pragma once
// File:   SimulationState.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <Robots/RobotModel.h>

// Types of selections in the simulation
enum class SelectionType {
	NONE,
	LINK,
	JOINT,
	ROBOT,
	OBJECT
	// may add more types later (e.g., JOINT, SENSOR, OBJECT)
};

enum class SelectionSource {
	NONE,
	CONTROL_PANEL,
	SCENE_VIEW
};

// Current selection state
struct Selection {
	SelectionType type = SelectionType::NONE;
	SelectionSource source = SelectionSource::NONE;
	int index = -1; // Index of the selected link or robot
};


