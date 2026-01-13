#include "pch.h"
#include "Interpreter/UIContext.h"
#include "EngineLib/LogMacros.h"

namespace commands {
	UIContext::UIContext(gui::simManager* sim) 
		: _sim(sim), _robot(&sim->getRobotModel()), _obj(nullptr), _defaultObj(nullptr) { }

	//  Set the colour property of the current object
	void UIContext::setColour(const glm::vec3& color) {
		_obj = _sim->getObject();

		if (_obj) {
			if (auto mesh = _obj->getMesh()) {
				mesh->_colour = color;
			}
		}
	}

	//  Set the roughness property of the current object
	void UIContext::setRoughness(float roughness) {
		_obj = _sim->getObject();

		if (_obj) {
			if (auto mesh = _obj->getMesh()) {
				mesh->_metallic = roughness;
			}
		}
	}

	//  Set the metallic property of the current object
	void UIContext::setMetallic(float metallic) {
		_obj = _sim->getObject();

		if (_obj) {
			if (auto mesh = _obj->getMesh()) {
				mesh->_metallic = metallic;
			}
		}
	}

	// --- OBJECT LOAD AND CLEAR METHODS ---

	void UIContext::loadObject(const std::string& objectPath) {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot load object.");
			return;
		}

		// Check if object already loaded (if so assigns new name)
		auto it = _loadedObjects.find(objectPath);
		if (it != _loadedObjects.end()) {
			_obj = it->second;
			LOG_INFO("Object already loaded from path: %s", objectPath.c_str());
			return;
		}

		// Load new object
		auto mesh = std::make_shared<scene::Mesh>();
		// Load mesh from file
		if (!mesh->load(objectPath)) {
			LOG_ERROR("Failed to load object from path: %s", objectPath.c_str());
			return;
		}
		// Initialize mesh (create buffers, etc.)
		auto newObj = new scene::Object(mesh);
		_sim->addObject(std::unique_ptr<scene::Object>(newObj));
		_loadedObjects[objectPath] = newObj;
		_obj = newObj;
		LOG_INFO("Loaded object from path: %s", objectPath.c_str());
	}

	void UIContext::clearObject() {
		_obj = _defaultObj;
	}

	// --- ROBOT LOAD AND CLEAR METHODS ---

	void UIContext::loadRobot(const std::string& robotName) {
		if (!_sim) {
			LOG_WARN("Simulation manager is null, cannot load robot.");
			return;
		}
		// Check if robot already loaded
		auto it = _loadedRobots.find(robotName);
		if (it != _loadedRobots.end()) {
			_robot = it->second;
			LOG_INFO("Robot already loaded: %s", robotName.c_str());
			return;
		}
		// Load new robot model
		_sim->loadRobot(robotName);

		auto& newRobot = _sim->getRobotModel();
		_loadedRobots[robotName] = &newRobot;
		_robot = &newRobot;
		LOG_INFO("Loaded robot model: %s", robotName.c_str());
	}

	void UIContext::clearRobot() {
		_robot = nullptr;
		_sim->clearRobot();
	}

	// --- TEXTURE LOAD AND CLEAR METHODS ---

	void UIContext::loadTexture(const std::string& texturePath) {
		if (!_obj) {
			LOG_WARN("No object selected to load texture onto.");
			return;
		}
		// Texture loading to be implemented at some point
	}

	void UIContext::clearTexture() {
		if (!_obj) {
			LOG_WARN("No object selected to clear texture from.");
			return;
		}
		// Texture clearing to be implemented at some point
	}

} // namespace commands
