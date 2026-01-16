#pragma once

#include "EngineCore.h"
#include "SimContext.h"
#include "Interpreter/Utils.h"


#include "Platform/Logger.h"

namespace scene { class Object; }
namespace interpreter { class StoredProgram; }

namespace commands {	
	class ENGINE_API UIContext {
	public:
		UIContext(gui::simManager* sim, scene::Object* obj);

		// Getters and Setters
		gui::simManager* getSim() const { return _sim; }
		scene::Object* getObject() const { return _obj; }
		void setObject(scene::Object* obj) { _obj = obj; }
		void setDefaultObject(scene::Object* obj) { _defaultObj = obj;  _obj = obj; }

		// setters for material properties
		utils::OpResult setColour(const glm::vec3& color);
		utils::OpResult setMetallic(float metallic);
		
		// loaders
		utils::OpResult loadObject(const std::string& objectPath);
		utils::OpResult loadRobot(const std::string& robotName);
		utils::OpResult loadTexture(const std::string& texturePath);
		
		// clearers
		utils::OpResult clearObject();
		utils::OpResult clearRobot();
		utils::OpResult clearTexture();

	private:
		gui::simManager* _sim;		// simulation manager
		RobotModel* _robot;			// current robot for commands
		scene::Object* _obj;		// current object for commands
		scene::Object* _defaultObj;	// default object for commands

		std::unordered_map<std::string, scene::Object*> _loadedObjects; // cache of loaded objects
		std::unordered_map<std::string, RobotModel*> _loadedRobots;		// cache of loaded robots
	};
}// namespace commands