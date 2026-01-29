#include "pch.h"
#include "Interpreter/UIContext.h"
#include "EngineLib/LogMacros.h"
#include "Scene/SimulationManager.h"
#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Scene/ObjectID.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

using namespace mathlib;
using namespace constants;
using namespace utils;

namespace commands {
	UIContext::UIContext(gui::simManager* sim, scene::ObjectID objID)
		: _sim(sim), _phys(sim ? &sim->getPhysicsSystem() : nullptr), _robot(sim ? sim->getRobotSystem() : nullptr),
		  _objID(objID), _defaultObjID(objID), _angularUnits(AngularUnits::DegPerSec) {
	}

	scene::ObjectID UIContext::DefaultObjectID() const { return _defaultObjID; }
	scene::ObjectID UIContext::ObjectID() const { return _objID; }

	scene::Object* UIContext::resolveObject(scene::ObjectID id) const {
		if (!_sim) return nullptr;
		if (id == scene::ObjectID::INVALID_OBJECT_ID) return nullptr;
		return _sim->getObjectByID(id);
	}

	scene::Object* UIContext::resolveCurrentObject() const { return resolveObject(_objID); }
	scene::Object* UIContext::resolveDefaultObject() const { return resolveObject(_defaultObjID); }

	OpResult UIContext::setOmega(const mathlib::Vec3& omega, AngularUnits units) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");

		Vec3 w = omega;
		if (units == AngularUnits::DegPerSec) { w *= (float)(PI / 180.0); }
		if (_omegaClamp > 0.0) {
			w.x() = (float)std::clamp((double)w.x(), -_omegaClamp, _omegaClamp);
			w.y() = (float)std::clamp((double)w.y(), -_omegaClamp, _omegaClamp);
			w.z() = (float)std::clamp((double)w.z(), -_omegaClamp, _omegaClamp);
		}
		
		obj->state.angularVelocity = w;
		return OpResult::Success(true);
	}

	OpResult UIContext::setFixedDt(double dt) {
		if (!_sim) { return OpResult::Failure("Simulation manager is null."); }
		if (dt <= 0.0) { return OpResult::Failure("Fixed dt must be positive."); }
		_sim->setFixedDeltaTime(dt);
		return OpResult::Success(true);
	}

	//  Set the colour property of the current object
	const OpResult UIContext::setColour(const glm::vec3& color) const {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");

		obj->setAlbedo(color);
		return OpResult::Success();
	}

	//  Set the metallic property of the current object
	const OpResult UIContext::setMetallic(float metallic) const {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");

		auto mesh = obj->getMesh();
		if (!mesh) return OpResult::Failure("Object has no mesh.");

		mesh->setMetallic(metallic);
		return OpResult::Success();
	}

	// --- OBJECT LOAD AND CLEAR METHODS ---

	OpResult UIContext::loadObject(const std::string& objectPath) {
		if (!_sim) { return OpResult::Failure("Simulation manager is null."); }
		if (objectPath.empty()) { return OpResult::Failure("Object path is empty."); }

		auto spawned = _sim->loadMeshReturn(objectPath);
		if (spawned.empty() || !spawned[0]) { return OpResult::Failure("No objects loaded from specified path."); }

		_objID = spawned[0]->id;
		_loadedObjects[objectPath] = _objID;
		return OpResult::Success(true);
	}

	// Removes the current object based on its index
	OpResult UIContext::clearObject() {
		if (!_sim) { return OpResult::Failure("Simulation manager is null."); }
		if (_objID == scene::ObjectID::INVALID_OBJECT_ID) { return OpResult::Failure("No object selected."); }

		// Finds the index of the current object
		auto& objects = _sim->getObjects();
		auto it = std::find_if(objects.begin(), objects.end(), [this](const std::unique_ptr<scene::Object>& o) { return o && o->id == _objID; });

		if (it == objects.end()) { return OpResult::Failure("Selected object ID not found."); }

		const int index = (int)std::distance(objects.begin(), it);
		const scene::ObjectID deletedId = _objID;

		_sim->deleteObject(index);

		// Clear context IDs safely
		_objID = scene::ObjectID::INVALID_OBJECT_ID;
		if (_defaultObjID == deletedId) {
			_defaultObjID = scene::ObjectID::INVALID_OBJECT_ID;
		}

		return OpResult::Success(true);
	}

	// --- ROBOT LOAD AND CLEAR METHODS ---

	OpResult UIContext::loadRobot(const std::string& robotName) {
		if (!_sim) { return OpResult::Failure("Simulation manager is null."); }
		if (robotName.empty()) return OpResult::Failure("Robot name is empty.");

		_sim->loadRobot(robotName);
		_robot = _sim->getRobotSystem();
		if (!_robot) return OpResult::Failure("Robot system is null after load.");
		return OpResult::Success(true);
	}

	OpResult UIContext::clearRobot() {
		if (!_sim) { return OpResult::Failure("Simulation manager is null."); }

		_robot = nullptr;
		_sim->clearRobot();
		return OpResult::Success(true);
	}

	// --- TEXTURE LOAD AND CLEAR METHODS ---
	// (Texture loading/clearing not implemented yet)

	const OpResult UIContext::loadTexture(const std::string& texturePath) const {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");
		return OpResult::Failure("Texture loading not implemented yet.");
	}

	const OpResult UIContext::clearTexture() const {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");
		return OpResult::Failure("Texture loading not implemented yet.");
	}

	// --- OBJECT SELECTION METHOD ---
	OpResult UIContext::selectObject(scene::ObjectID id) {
		scene::Object* obj = resolveObject(id);
		if (!obj) return OpResult::Failure("Object ID not found.");
		_objID = id;
		return OpResult::Success();
	}

	// --- PRIMARY SIMULATION COMMANDS ---
	OpResult UIContext::startSim() {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot start simulation.");
			return OpResult::Failure("Simulation manager is null.");
		}
		_sim->startSimulation();
		return OpResult::Success();
	}
} // namespace commands
