#pragma once

// ============================================
//            File: RobotLoader.h
// ============================================
// Class responsible for loading robotic models from JSON files.
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Robots/RobotModel.h"

namespace robots {
	class ENGINE_API RobotLoader {
	public:
		static RobotModel loadFromJSON(const std::string& filepath);
	};
}