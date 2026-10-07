/*
 * File: Platform/SimulationState.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <Systems/RigidBodyModel.h>

// Types of selections in the simulation
enum class SelectionType {
	NONE,
	LINK,
	JOINT,
	ROBOT,
	FREE_BODY,
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

// Struct to hold comparison results for integrator analysis
struct ComparisonSnapshot {
	std::string integratorName;
	std::vector<float> time;      // time samples
	std::vector<float> errRms;    // RMS error time series
	std::vector<float> errMax;    // Max error time series
	std::vector<std::vector<float>> jointErr; // [joint][sample]
	int jointCount = 0;
};