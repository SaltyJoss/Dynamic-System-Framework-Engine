// DSFE_Core RobotSimSnapshot.h
#pragma once

#include "EngineCore.h"
#include "Robots/RobotModel.h"

namespace robots {
	// Immutable robot data needed by solver threads
	struct DSFE_API RobotConstModel {
		std::string name;
		bool baseFrameIsAligned = false;
		double scale = 1.0;
		mathlib::Mat4 baseFrame = mathlib::Mat4::Identity();

		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;
	};

	// Runtime snapshot for one integration/derivative step
	struct DSFE_API RobotSimSnapshot {
		const RobotConstModel* model = nullptr;

		mathlib::VecX q;   // joint angles
		mathlib::VecX qd;  // joint velocities
		mathlib::VecX eta; // joint control states (e.g. for muscle models)

		mathlib::VecX q_ref;   // reference joint angles
		mathlib::VecX qd_ref;  // reference joint velocities
		mathlib::VecX qdd_ref; // reference joint accelerations

		mathlib::Mat4 robotRootPose = mathlib::Mat4::Identity();

		bool baseIsFree = false;
		double gravity = 0.0;
		eTorqueMode torqueMode = eTorqueMode::CONTROLLED;
		double simTime = 0.0; // simulation time in seconds
	};
} // namespace robots