#include "pch.h"
#include "Interpreter/UIContext.h"
#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace utils;

namespace commands {
	UIContext::UIContext(gui::simManager* sim, scene::Object* obj)
		: _sim(sim), _robot(&sim->getRobotModel()), _obj(obj), _defaultObj(obj) { }

	//  Set the colour property of the current object
	OpResult UIContext::setColour(const glm::vec3& color) {
		_obj = _sim->getObject();

		if (_obj) {
			if (auto mesh = _obj->getMesh()) {
				mesh->_colour = color;
				return OpResult::Success();
			}
			return OpResult::Failure("Object has no mesh.");
		}
		return OpResult::Failure("No object selected.");
	}

	//  Set the metallic property of the current object
	OpResult UIContext::setMetallic(float metallic) {
		_obj = _sim->getObject();

		if (_obj) {
			if (auto mesh = _obj->getMesh()) {
				mesh->_metallic = metallic;
				return OpResult::Success();
			}
			return OpResult::Failure("Object has no mesh.");
		}
		return OpResult::Failure("No object selected.");
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
			_obj = it->second;
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
		_obj = objects.back().get(); // get the last loaded object
		_loadedObjects[objectPath] = _obj;

		LOG_INFO("Loaded object from path: %s", objectPath.c_str());
		return OpResult::Success();
	}

	// Removes the current object based on its index in the simulation manager
	OpResult UIContext::clearObject() {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot clear object.");
			return OpResult::Failure("Simulation manager is null.");
		}
		if (!_obj) {
			LOG_WARN("No object selected to clear.");
			return OpResult::Failure("No object selected.");
		}

		// Find the index of the current object in the simulation manager
		auto& objects = _sim->getObjects();
		auto it = std::find_if(objects.begin(), objects.end(), [this](const std::unique_ptr<scene::Object>& o) { return o.get() == _obj; });

		// If found, delete the objectS
		if (it != objects.end()) {
			size_t index = std::distance(objects.begin(), it);
			_sim->deleteObject(static_cast<int>(index));
			_obj = nullptr;
			LOG_INFO("Cleared selected object.");
		} else {
			LOG_WARN("Selected object not found in simulation manager.");
			return OpResult::Failure("Selected object not found.");
		}
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

		auto& newRobot = _sim->getRobotModel();
		_loadedRobots[robotName] = &newRobot;
		_robot = &newRobot;
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
		if (!_obj) {
			LOG_WARN("No object selected to load texture onto.");
			return OpResult::Failure("No object selected.");
		}
		// Texture loading to be implemented at some point
		return OpResult::Failure("Texture loading not implemented yet.");
	}

	OpResult UIContext::clearTexture() {
		if (!_obj) {
			LOG_WARN("No object selected to clear texture from.");
			return OpResult::Failure("No object selected.");
		}
		// Texture clearing to be implemented at some point
		return OpResult::Failure("Texture clearing not implemented yet.");
	}
} // namespace commands
