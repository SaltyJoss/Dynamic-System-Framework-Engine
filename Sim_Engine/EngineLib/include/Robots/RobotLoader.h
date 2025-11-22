#pragma once

#include "EngineCore.h"
#include "Robots/RobotModel.h"

namespace robots {
	class ENGINE_API RobotLoader {
	public:
		static RobotModel loadFromJSON(const std::string& filepath);
	};
}