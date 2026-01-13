#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "Scene/SimulationManager.h"

#include "Platform/Logger.h"

using namespace mathlib;

namespace scene { class Object; }
namespace interpreter { class StoredProgram; }

namespace commands {
	// Struct for operation result
	struct OpResult {
		bool ok = true;
		std::string message;
		static OpResult Success() { return { true, {} }; }
		static OpResult Failure(const std::string& msg) { return OpResult{ false, msg }; }
	};
	
	class ENGINE_API UIContext {
	public:
		UIContext(gui::simManager* sim);

		// Getters and Setters
		gui::simManager* getSim() const { return _sim; }
		scene::Object* getObject() const { return _obj; }
		void setObject(scene::Object* obj) { _obj = obj; }
		void setDefaultObject(scene::Object* obj) { _defaultObj = obj;  _obj = obj; }

		// setters for material properties
		void setColour(const glm::vec3& color);
		void setRoughness(float roughness);
		void setMetallic(float metallic);
		
		// loaders
		void loadObject(const std::string& objectPath);
		void loadRobot(const std::string& robotName);
		void loadTexture(const std::string& texturePath);
		
		// clearers
		void clearObject();
		void clearRobot();
		void clearTexture();

	private:
		gui::simManager* _sim;		// simulation manager
		RobotModel* _robot;			// current robot for commands
		scene::Object* _obj;		// current object for commands
		scene::Object* _defaultObj;	// default object for commands

		std::unordered_map<std::string, scene::Object*> _loadedObjects; // cache of loaded objects
		std::unordered_map<std::string, RobotModel*> _loadedRobots;		// cache of loaded robots
	};
}// namespace commands