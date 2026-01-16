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

		// Check if object already loaded (if so assigns new name)
		auto it = _loadedObjects.find(objectPath);
		if (it != _loadedObjects.end()) {
			_obj = it->second;
			LOG_INFO("Object already loaded from path: %s", objectPath.c_str());
			return OpResult::Success();
		}

		// Load new object
		auto mesh = std::make_shared<scene::Mesh>();
		// Load mesh from file
		if (!mesh->load(objectPath)) {
			LOG_ERROR("Failed to load object from path: %s", objectPath.c_str());
			return OpResult::Failure("Failed to load object mesh.");
		}
		// Initialize mesh (create buffers, etc.)
		auto newObj = new scene::Object(mesh);
		_sim->addObject(std::unique_ptr<scene::Object>(newObj));
		_loadedObjects[objectPath] = newObj;
		_obj = newObj;

		LOG_INFO("Loaded object from path: %s", objectPath.c_str());
		return OpResult::Success();
	}

	OpResult UIContext::clearObject() {
		_obj = _defaultObj;
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
