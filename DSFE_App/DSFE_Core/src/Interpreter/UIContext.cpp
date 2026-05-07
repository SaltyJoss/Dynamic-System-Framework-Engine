// DSFE_Core UIContext.cpp
#include "pch.h"

#include "Interpreter/UIContext.h"

#include "Scene/SimulationCore.h"
#include "Robots/RobotSystem.h"

#include "Platform/DataManager.h"
#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;
using namespace utils;

namespace commands {
	// Constructor
	UIContext::UIContext(core::ISimulationCore* core)
		: _core(core), _robot(core ? core->robotSystem() : nullptr), 
		_angularUnits(AngularUnits::DegPerSec) {
	}

	// --- OBJECT RESOLUTION METHODS ---
	//scene::ObjectID UIContext::DefaultObjectID() const { return _defaultObjID; }
	//scene::ObjectID UIContext::ObjectID() const { return _objID; }

	//scene::Object* UIContext::resolveObject(scene::ObjectID id) const {
	//	if (!_core) return nullptr;
	//	if (id == scene::ObjectID::INVALID_OBJECT_ID) return nullptr;
	//	return _core->getObjectByID(id);
	//}

	//scene::Object* UIContext::resolveCurrentObject() const { return resolveObject(_objID); }
	//scene::Object* UIContext::resolveDefaultObject() const { return resolveObject(_defaultObjID); }

	// --- GLOBAL STATE METHODS ---
	
	//// Set the angular velocity of the current object, with unit conversion and optional clamping
	//OpResult UIContext::setOmega(const mathlib::Vec3& omega, AngularUnits units) {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) return OpResult::Failure("No object selected.");

	//	Vec3 w = omega;
	//	if (units == AngularUnits::DegPerSec) { w *= (float)(PI / 180.0); }
	//	if (_omegaClamp > 0.0) {
	//		w.x() = (float)std::clamp((double)w.x(), -_omegaClamp, _omegaClamp);
	//		w.y() = (float)std::clamp((double)w.y(), -_omegaClamp, _omegaClamp);
	//		w.z() = (float)std::clamp((double)w.z(), -_omegaClamp, _omegaClamp);
	//	}
	//	
	//	obj->state.angularVelocity = w;
	//	return OpResult::Success(true);
	//}

	// Set the fixed delta time for the simulation
	OpResult UIContext::setFixedDt(double dt) {
		if (!_core) { return OpResult::Failure("Simulation manager is null."); }
		if (dt <= 0.0) { return OpResult::Failure("Fixed dt must be positive."); }
		_core->setFixedDt(dt);
		return OpResult::Success(true);
	}

	//  Set the colour property of the current object
	//const OpResult UIContext::setColour(const glm::vec3& color) const {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) return OpResult::Failure("No object selected.");

	//	obj->setAlbedo(color);
	//	return OpResult::Success();
	//}

	//  Set the metallic property of the current object
	//const OpResult UIContext::setMetallic(float metallic) const {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) return OpResult::Failure("No object selected.");

	//	auto mesh = obj->getMesh();
	//	if (!mesh) return OpResult::Failure("Object has no mesh.");

	//	mesh->setMetallic(metallic);
	//	return OpResult::Success();
	//}

	// --- OBJECT LOAD AND CLEAR METHODS ---

	// Loads a new object from the specified file path and updates the context with the new object's ID
	//OpResult UIContext::loadObject(const std::string& objectPath) {
	//	if (!_core) { return OpResult::Failure("Simulation manager is null."); }
	//	if (objectPath.empty()) { return OpResult::Failure("Object path is empty."); }

	//	auto spawned = _core->loadMeshReturn(objectPath);
	//	if (spawned.empty() || !spawned[0]) { return OpResult::Failure("No objects loaded from specified path."); }

	//	_objID = spawned[0]->id;
	//	_loadedObjects[objectPath] = _objID;
	//	return OpResult::Success(true);
	//}

	// Removes the current object based on its index
	//OpResult UIContext::clearObject() {
	//	if (!_core) { return OpResult::Failure("Simulation manager is null."); }
	//	if (_objID == scene::ObjectID::INVALID_OBJECT_ID) { return OpResult::Failure("No object selected."); }

	//	// Finds the index of the current object
	//	auto& objects = _core->getObjects();
	//	auto it = std::find_if(objects.begin(), objects.end(), [this](const std::unique_ptr<scene::Object>& o) { return o && o->id == _objID; });

	//	if (it == objects.end()) { return OpResult::Failure("Selected object ID not found."); }

	//	const int index = (int)std::distance(objects.begin(), it);
	//	const scene::ObjectID deletedId = _objID;

	//	_core->deleteObject(index);

	//	// Clear context IDs safely
	//	_objID = scene::ObjectID::INVALID_OBJECT_ID;
	//	if (_defaultObjID == deletedId) {
	//		_defaultObjID = scene::ObjectID::INVALID_OBJECT_ID;
	//	}

	//	return OpResult::Success(true);
	//}

	// --- ROBOT LOAD AND CLEAR METHODS ---

	// Loads a robot by name and updates the context with the new robot system
	OpResult UIContext::loadRobot(const std::string& robotName) {
		if (!_core) { return OpResult::Failure("Simulation manager is null."); }
		if (robotName.empty()) return OpResult::Failure("Robot name is empty.");

		_core->loadRobot(robotName);
		_robot = _core->robotSystem();
		if (!_robot) return OpResult::Failure("Robot system is null after load.");
		return OpResult::Success(true);
	}

	// --- TEXTURE LOAD AND CLEAR METHODS ---
	// (Texture loading/clearing not implemented yet)

	// Loads a texture from the specified file path and applies it to the current object
	//const OpResult UIContext::loadTexture(const std::string& /*texturePath*/) const {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) return OpResult::Failure("No object selected.");
	//	return OpResult::Failure("Texture loading not implemented yet.");
	//}

	// Clears the texture from the current object
	//const OpResult UIContext::clearTexture() const {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) return OpResult::Failure("No object selected.");
	//	return OpResult::Failure("Texture loading not implemented yet.");
	//}

	// --- OBJECT SELECTION METHOD ---

	// Selects an object by its ID and updates the context with the new selected object ID
	//OpResult UIContext::selectObject(scene::ObjectID id) {
	//	scene::Object* obj = resolveObject(id);
	//	if (!obj) return OpResult::Failure("Object ID not found.");
	//	_objID = id;
	//	return OpResult::Success();
	//}

	// --- PRIMARY SIMULATION COMMANDS ---

	// Starts the simulation if the simulation manager is available
	OpResult UIContext::startSim() {
		if (!_core) {
			LOG_WARN("Simulation manager is null, cannot start simulation.");
			return OpResult::Failure("Simulation manager is null.");
		}
		_core->startSimulation();
		return OpResult::Success();
	}
} // namespace commands
