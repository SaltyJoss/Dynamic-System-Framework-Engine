#pragma once

// ============================================
//            File: RobotLoader.h
// ============================================
// Class responsible for loading robotic models from JSON files.
//
// Summary:
// ============================================
//
// public:
// --------------------------------------------
// RobotModel loadFromJSON(const std::string& filepath)
//      -> Loads a robotic model from the specified JSON file path and returns a RobotModel object.
// --------------------------------------------
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