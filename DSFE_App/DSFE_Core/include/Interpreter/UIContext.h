// DSFE_Core UIContext.h
#pragma once

#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/Utils.h"
#include <glm/glm.hpp>

#include "Platform/Logger.h"

// Forward declarations
namespace interpreter { class DSFE_API StoredProgram; }

namespace commands {	
	class DSFE_API UIContext {
	public:
		UIContext(core::ISimulationCore* core);

		core::ISimulationCore* Core() const { return _core; }
		robots::RobotSystem* Robot() const { return _robot; }
		scene::ObjectID DefaultObjectID() const;
		scene::ObjectID ObjectID() const;

		scene::Object* resolveObject(scene::ObjectID id) const;
		scene::Object* resolveCurrentObject() const;
		scene::Object* resolveDefaultObject() const;

		// Setters (setCmd)
		utils::OpResult setOmega(const mathlib::Vec3& omega, utils::AngularUnits units);
		utils::OpResult setFixedDt(double dt);

		/*const utils::OpResult setColour(const glm::vec3& color) const;*/
		const utils::OpResult setMetallic(float metallic) const;
		
		// Loaders (loadCmd)
		utils::OpResult loadObject(const std::string& objectPath);
		utils::OpResult loadRobot(const std::string& robotName);
		const utils::OpResult loadTexture(const std::string& texturePath) const;
		
		// Clearers (clearCmd)
		utils::OpResult clearObject();
		const utils::OpResult clearTexture() const;

		// Selectors (selectCmd)
		utils::OpResult selectObject(scene::ObjectID id);

		utils::OpResult startSim();

	private:
		core::ISimulationCore* _core = nullptr;
		physics::PhysicsSystem* _phys = nullptr;
		robots::RobotSystem* _robot = nullptr;
		scene::ObjectID _objID;
		scene::ObjectID _defaultObjID;

		utils::AngularUnits _angularUnits = utils::AngularUnits::DegPerSec;
		double _omegaClamp = 0.0; // Default: no clamp

		std::unordered_map<std::string, scene::ObjectID> _loadedObjects; // cache IDs, not pointers
		std::unordered_map<std::string, robots::RobotSystem*> _loadedRobots;
	};
}// namespace commands