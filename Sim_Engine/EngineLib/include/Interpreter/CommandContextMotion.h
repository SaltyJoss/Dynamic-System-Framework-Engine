#pragma once

#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/Utils.h"

#include "Platform/Logger.h"

namespace commands {
	struct ENGINE_API ActiveRigidRot {
		scene::Object* obj = nullptr;
		mathlib::Vec3 axisUnit{ 0.0, 0.0, 0.0 };
		mathlib::Quat qStart{ 1.0, 0.0, 0.0, 0.0 };
		mathlib::Quat qTarget{ 1.0, 0.0, 0.0, 0.0 };
		double maxOmega = 0.0; // rad/s
		double epsAngle = 0.5 * constants::PI / 180; // rad
		bool active = false;
	};

	struct ENGINE_API ActiveJointRot {
		std::string link;
		double start = 0.0;
		double target = 0.0; // rad
		double maxOmega = 0.0; // rad/s
		double epsAngle = 0.5 * constants::PI / 180; // rad
		bool wrapShortest = true;
		bool active = false;
	};

	// Class representing the command context
	class ENGINE_API CommandContextMotion {
	public:
		CommandContextMotion(gui::simManager* sim, scene::ObjectID objID);

		// --- GLOBAL STATE METHODS ---

		// Sets the angular units for rotation commands
		void setAngularUnits(utils::AngularUnits units);
		// Gets the current angular units
		utils::AngularUnits getAngularUnits() const;

		// Sets the maximum absolute angular velocity (omega) clamp
		void setOmegaClamp(double maxAbsOmega);
		// Gets the current omega clamp value
		double getOmegaClamp() const;

		// Sets the angular velocity (omega) for the current object
		utils::OpResult setOmega(const mathlib::Vec3& omega); // rad/s
		utils::OpResult stopAllOmega(); // stops all angular velocity

		utils::OpResult setJointOmega(const std::string& childLink, double omegaDegPerSec); // deg/s

		// --- HELPER METHODS ---



		gui::simManager* Sim() const { return _sim; }
		robots::RobotSystem* Robot() const { return _robot; }
		scene::ObjectID DefaultObjectID() const;
		scene::ObjectID ObjectID() const;

		scene::Object* resolveObject(scene::ObjectID id) const;
		scene::Object* resolveCurrentObject() const;
		scene::Object* resolveDefaultObject() const;

		void setDefaultObjectID(scene::ObjectID id) { _defaultObjID = id; _objID = id; }
		void setObjectID(scene::ObjectID id) { _objID = id; }

		// --- PROCESS CONTROL METHODS ---
	
		// Stop Motion
		void stopRotation(scene::Object* obj, utils::AxisMask axes);
		void stopTranslation(scene::Object* obj, utils::AxisMask axes);

		// --- ROTATION COMMAND METHODS ---

		// Rotates an object around specified axes at a given angular velocity
		utils::OpResult rotateObject(scene::Object* obj, utils::AxisMask axes, double omega, double dt);
		// Rotates specified axes at a given angular velocity
		utils::OpResult rotateAxes(utils::AxisMask axes, double omega, double dt);

		utils::OpResult setJointTargetRad(const std::string& link, double thetaTargetRad);
		utils::OpResult setJointTargetDeltaRad(const std::string& link, double deltaRad);
		utils::OpResult setJointMaxOmegaRad(const std::string& link, double maxOmegaRad_s);
		utils::OpResult setJointOmegaRefRad(const std::string& link, double omegaRefRad_s);
		utils::OpResult setJointAlphaRefRad(const std::string& link, double alphaRefRad_s2);

		utils::OpResult updateRigidRotateTo(double dt);
		utils::OpResult updateJointRotateTo(double dt);

		utils::OpResult beginRigidRotateTo(scene::Object* obj, mathlib::Vec3 axisUnit, double maxOmegaDegPerSec, double angleDeg);
		utils::OpResult beginJointRotateTo(const std::string& link, double maxOmegaDegPerSec, double angleDeg);

		// --- TRANSLATION COMMAND METHODS ---

		// Translates in world coordinates along a specified direction
		utils::OpResult translateWorld(const mathlib::Vec3& direction, double distance, double vel);
		// Translates along specified axes at a given velocity
		const utils::OpResult translateAxes(utils::AxisMask axes, double vel, double dt) const;

		// --- READ-ONLY ACCESSORS ---
		bool hasLink(std::size_t linkIndex) const;


	private:
		gui::simManager* _sim = nullptr;
		physics::PhysicsSystem* _phys = nullptr;
		robots::RobotSystem* _robot = nullptr;
		scene::ObjectID _objID;
		scene::ObjectID _defaultObjID;

		utils::AngularUnits _angularUnits = utils::AngularUnits::DegPerSec;
		double _omegaClamp = 0.0; // Default: no clamp

		ActiveRigidRot _rig;
		ActiveJointRot _jnt;

		double NormaliseOmega(double omega) const;
		double convertOmegaToInternal(double omega) const;

		mathlib::Vec3 normaliseDirection(const mathlib::Vec3& dir) const;

		double getJointAngleRad(const std::string& link) const;

		std::unordered_map<std::string, float> _jointAngles;

		float _currentAngle = 0.0; // current angle for rotation commands
		std::string _currentLinkName; // current link name for joint commands

		mathlib::Vec3 angularVelocityPrev = mathlib::Vec3::Zero();
		mathlib::Vec3 linearVelocityPrev = mathlib::Vec3::Zero();

		double _dtheta = 0.0; // angle displacement
		double _dt = 0.0; // time interval
		double _omega = 0.0; // angular velocity
	};
} // namespace commands