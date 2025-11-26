#pragma once

// ============================================
// 		File: SimulationState.h
// ============================================
// Structs and enums representing the current state of selections in the simulation.
//
// Summary:
// ============================================
// structs / enumerations:
// --------------------------------------------
// SelectionType
//      -> Enumeration of different types of selections in the simulation (NONE, LINK, ROBOT
// Selection
//      -> Struct representing the current selection state, including type and index.
// --------------------------------------------
//
// ============================================

#include "EngineCore.h"
#include <Robots/RobotModel.h>

// Types of selections in the simulation
enum class SelectionType {
	NONE,
	LINK,
	ROBOT,
	OBJECT
	// may add more types later (e.g., JOINT, SENSOR, OBJECT)
};

// Current selection state
struct Selection {
	SelectionType type = SelectionType::NONE;
	int index = -1; // Index of the selected link or robot
};


