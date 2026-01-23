#pragma once

// ============================================
// 		File: SimulationState.h
// ============================================
// Structs and enums representing the current state of selections in the simulation.
//
// Summary:
// ============================================
// 
// structs / enumerations:
// --------------------------------------------
// enum class SelectionType
//      -> Enumeration of different types of selections in the simulation (NONE, LINK, ROBOT, OBJECT).
// enum class SelectionSource
//      -> Enumeration of different sources of selection (NONE, CONTROL_PANEL, SCENE_VIEW).
// struct Selection
//      -> Struct representing the current selection state, including type and index.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

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
	// may add more sources later (e.g., HIERARCHY_VIEW, PROPERTIES_PANEL)
};

// Current selection state
struct Selection {
	SelectionType type = SelectionType::NONE;
	SelectionSource source = SelectionSource::NONE;
	int index = -1; // Index of the selected link or robot
};


