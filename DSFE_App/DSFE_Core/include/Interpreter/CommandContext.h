// DSFE_Core CommandContext.h
#pragma once

#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/Utils.h"

#include "Platform/Logger.h"

namespace commands {
	struct DSFE_API ActiveRigidRot {
		mathlib::Vec3 axisUnit{ 0.0, 0.0, 0.0 };
		mathlib::Quat qStart{ 1.0, 0.0, 0.0, 0.0 };
		mathlib::Quat qTarget{ 1.0, 0.0, 0.0, 0.0 };
		double maxOmega = 0.0; // rad/s
		double epsAngle = 0.5 * constants::PI / 180; // rad
		bool active = false;
	};

	struct DSFE_API ActiveJointRot {
		std::string link;
		double start = 0.0;
		double target = 0.0; // rad
		double maxOmega = 0.0; // rad/s
		double epsAngle = 0.5 * constants::PI / 180; // rad
		bool wrapShortest = true;
		bool active = false;
	};

	// Class representing the command context
	class DSFE_API CommandContext {
	public:
		CommandContext(core::ISimulationCore* core);

		// --- INITIALISATION METHODS ---
		utils::OpResult startSim();
		utils::OpResult setFixedDt(double dt);
		utils::OpResult loadSingleBody(const std::string& bodyName);
		utils::OpResult loadMultibody(const std::string& bodyName);

		// --- GLOBAL STATE METHODS ---

		// Sets the angular units for rotation commands
		void setAngularUnits(utils::AngularUnits units);
		// Gets the current angular units
		utils::AngularUnits getAngularUnits() const;

		// Sets the maximum absolute angular velocity (omega) clamp
		void setOmegaClamp(double maxAbsOmega);
		// Gets the current omega clamp value
		double getOmegaClamp() const;

		// Stops all angular velocity for the robot
		utils::OpResult stopAllOmega(); // stops all angular velocity

		utils::OpResult setJointOmega(const std::string& childLink, double omegaDegPerSec); // deg/s
		utils::OpResult stopJointOmega(const std::string& childLink);

		// --- HELPER METHODS ---
		core::ISimulationCore* Core() const { return _core; }
		robots::RobotSystem* Robot() const { return _robot; }


		// --- ROTATION COMMAND METHODS ---

		utils::OpResult setJointTargetRad(const std::string& link, double thetaTargetRad);
		utils::OpResult setJointTargetDeltaRad(const std::string& link, double deltaRad);
		utils::OpResult setJointMaxOmegaRad(const std::string& link, double maxqd);
		utils::OpResult setJointOmegaRefRad(const std::string& link, double qd_ref);
		utils::OpResult setJointAlphaRefRad(const std::string& link, double qdd_ref);

		//utils::OpResult updateRigidRotateTo(double dt);
		utils::OpResult updateJointRotateTo(double dt);

		//utils::OpResult beginRigidRotateTo(scene::Object* obj, mathlib::Vec3 axisUnit, double maxOmegaDegPerSec, double angleDeg);
		utils::OpResult beginJointRotateTo(const std::string& link, double maxOmegaDegPerSec, double angleDeg);

		// --- READ-ONLY ACCESSORS ---
		bool hasLink(std::size_t linkIndex) const;

	private:
		core::ISimulationCore* _core = nullptr;
		robots::RobotSystem* _robot = nullptr;

		utils::AngularUnits _angularUnits = utils::AngularUnits::DegPerSec;
		double _omegaClamp = 0.0; // Default: no clamp

		ActiveRigidRot _rig;
		ActiveJointRot _jnt;

		double NormaliseOmega(double omega) const;
		double convertOmegaToInternal(double omega) const;

		mathlib::Vec3 normaliseDirection(const mathlib::Vec3& dir) const;

		double getJointAngleRad(const std::string& link) const;

		std::unordered_map<std::string, double> _jointAngles;

		double _currentAngle = 0.0; // current angle for rotation commands
		std::string _currentLinkName; // current link name for joint commands

		mathlib::Vec3 angularVelocityPrev = mathlib::Vec3::Zero();
		mathlib::Vec3 linearVelocityPrev = mathlib::Vec3::Zero();

		double _dtheta = 0.0; // angle displacement
		double _dt = 0.0; // time interval
		double _omega = 0.0; // angular velocity
	};
} // namespace commands