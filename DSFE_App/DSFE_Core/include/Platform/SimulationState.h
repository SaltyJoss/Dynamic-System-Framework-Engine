// DSFE_Core SimulationState.h
#pragma once

#include "EngineCore.h"
#include <Robots/RobotModel.h>

// Types of selections in the simulation
enum class SelectionType {
	NONE,
	LINK,
	JOINT,
	ROBOT,
	BODY
	// may add more types later (e.g., JOINT, SENSOR, OBJECT)
};

enum class SelectionSource {
	NONE,
	CONTROL_PANEL,
	SCENE_VIEW
};

// Current selection state
struct DSFE_API Selection {
	SelectionType type = SelectionType::NONE;
	SelectionSource source = SelectionSource::NONE;
	int index = -1; // Index of the selected link or robot
};

// Current simulation run mode (e.g., interactive with real-time rendering vs. synchronous for
enum class eRunMode {
	Interactive,
	Synchronous
};

// Struct to hold the current simulation mode and related settings
struct DSFE_API modes {
	eRunMode _runMode = eRunMode::Interactive;
};


