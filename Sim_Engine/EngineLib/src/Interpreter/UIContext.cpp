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

	scene::ObjectID UIContext::getDefaultObjectID() const { return _defaultObjID; }
	scene::ObjectID UIContext::getObjectID() const { return _objID; }

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
		if (!_phys) {
			LOG_WARN("Physics system is null, cannot set fixed dt.");
			return OpResult::Failure("Physics system is null.");
		}
		if (dt <= 0.0) {
			LOG_WARN("Invalid fixed dt value: %f", dt);
			return OpResult::Failure("Fixed dt must be positive.");
		}
		_sim->setFixedDeltaTime(dt);
		return OpResult::Success(true);
	}

	//  Set the colour property of the current object
	OpResult UIContext::setColour(const glm::vec3& color) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");

		obj->setAlbedo(color);
		return OpResult::Success();
	}

	//  Set the metallic property of the current object
	OpResult UIContext::setMetallic(float metallic) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");

		auto mesh = obj->getMesh();
		if (!mesh) return OpResult::Failure("Object has no mesh.");

		mesh->setMetallic(metallic);
		return OpResult::Success();
	}

	// --- OBJECT LOAD AND CLEAR METHODS ---

	OpResult UIContext::loadObject(const std::string& objectPath) {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot load object.");
			return OpResult::Failure("Simulation manager is null.");
		}

		if (objectPath.empty()) {
			LOG_WARN("Empty object path provided -> %s", objectPath);
			return OpResult::Failure("Object path is empty.");
		}

		// Check if object already loaded (if so assigns new name)
		auto it = _loadedObjects.find(objectPath);
		if (it != _loadedObjects.end()) {
			_objID = it->second;
			LOG_INFO("Object already loaded from path: %s", objectPath.c_str());
			return OpResult::Success();
		}

		// Load new mesh from file
		_sim->loadMesh(objectPath);
		auto& objects = _sim->getObjects();
		if (objects.empty()) {
			LOG_WARN("No objects loaded from path: %s", objectPath.c_str());
			return OpResult::Failure("No objects loaded from the specified path.");
		}

		scene::Object* obj = objects.back().get();
		if (!obj) return OpResult::Failure("Loaded object is null.");

		_objID = obj->id;
		_loadedObjects[objectPath] = _objID;

		LOG_INFO("Loaded object from path: %s", objectPath.c_str());
		return OpResult::Success();
	}

	// Removes the current object based on its index
	OpResult UIContext::clearObject() {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot clear object.");
			return OpResult::Failure("Simulation manager is null.");
		}
		if (_objID == scene::ObjectID::INVALID_OBJECT_ID) {
			LOG_WARN("No object selected to clear.");
			return OpResult::Failure("No object selected.");
		}

		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("Selected object ID not found.");

		// Finds the index of the current object
		auto& objects = _sim->getObjects();
		auto it = std::find_if(objects.begin(), objects.end(), [this](const std::unique_ptr<scene::Object>& o) { return o && o->id == _objID; });

		// If found, delete the objects
		if (it == objects.end()) {
			LOG_WARN("Selected object not found in simulation manager.");
			return OpResult::Failure("Selected object not found.");
		}

		size_t index = std::distance(objects.begin(), it);
		_sim->deleteObject(static_cast<int>(index));

		_objID = scene::ObjectID::INVALID_OBJECT_ID;
		return OpResult::Success();
	}

	// --- ROBOT LOAD AND CLEAR METHODS ---

	OpResult UIContext::loadRobot(const std::string& robotName) {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot load robot.");
			return OpResult::Failure("Simulation manager is null.");
		}
		// Check if robot already loaded
		auto it = _loadedRobots.find(robotName);
		if (it != _loadedRobots.end()) {
			_robot = it->second;
			LOG_INFO("Robot already loaded: %s", robotName.c_str());
			return OpResult::Failure("Robot already loaded.");
		}

		// Load new robot model
		_sim->loadRobot(robotName);

		robots::RobotSystem* newRobot = _sim->getRobotSystem();
		_loadedRobots[robotName] = newRobot;
		_robot = newRobot;
		LOG_INFO("Loaded robot model: %s", robotName.c_str());
		return OpResult::Success();
	}

	OpResult UIContext::clearRobot() {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot clear robot.");
			return OpResult::Failure("Simulation manager is null.");
		}
		if (!_robot) {
			LOG_WARN("No robot loaded to clear.");
			return OpResult::Failure("No robot loaded.");
		}

		_robot = nullptr;
		_sim->clearRobot();
		return OpResult::Success();
	}

	// --- TEXTURE LOAD AND CLEAR METHODS ---
	// (Texture loading/clearing not implemented yet)

	OpResult UIContext::loadTexture(const std::string& texturePath) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) return OpResult::Failure("No object selected.");
		return OpResult::Failure("Texture loading not implemented yet.");
	}

	OpResult UIContext::clearTexture() {
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
