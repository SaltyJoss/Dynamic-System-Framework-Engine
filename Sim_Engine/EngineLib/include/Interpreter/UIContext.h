#pragma once

#include "EngineCore.h"
#include "SimContext.h"
#include "Interpreter/Utils.h"


#include "Platform/Logger.h"

namespace scene { class ENGINE_API Object; }
namespace interpreter { class ENGINE_API StoredProgram; }

namespace commands {	
	class ENGINE_API UIContext {
	public:
		UIContext(gui::simManager* sim, scene::ObjectID obj);

		// Getters and Setters
		gui::simManager* getSim() const { return _sim; }

		scene::ObjectID getObjectID() const { return _objID; }
		void setObjectID(scene::ObjectID id) { _objID = id; }

		scene::ObjectID getDefaultObjectID() const { return _defaultObjID; }
		void setDefaultObjectID(scene::ObjectID id) { _defaultObjID = id; _objID = id; }

		scene::Object* resolveObject() const;

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
		scene::ObjectID _objID = scene::INVALID_OBJECT_ID;
		scene::ObjectID _defaultObjID = scene::INVALID_OBJECT_ID;

		std::unordered_map<std::string, scene::ObjectID> _loadedObjects; // cache IDs, not pointers
		std::unordered_map<std::string, RobotModel*> _loadedRobots;
	};
}// namespace commands