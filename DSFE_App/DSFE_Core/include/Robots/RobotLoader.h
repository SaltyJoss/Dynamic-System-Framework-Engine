// DSFE_Core RobotLoader.h
#pragma once

#include "EngineCore.h"
#include "Robots/RobotModel.h"

namespace robots {
	class DSFE_API RobotLoader {
	public:
		static RobotModel loadFromJSON(const std::string& filepath);
	};
} // namespace robots