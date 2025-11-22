#pragma once

#include "EngineCore.h"
#include "Scene/Object.h"

#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

struct RobotLink {
	std::string name;
	std::string meshFile;
	elements::Object* attachedObject = nullptr;
};

struct RobotJoint {
	std::string name;
	std::string parent;
	std::string child;
	
	glm::vec3 axis;
	glm::vec3 offset;
	float angle = 0.0f;
};

struct RobotModel {
	float scale = 1.0f;
	std::vector<RobotLink> links;
	std::vector<RobotJoint> joints;
};