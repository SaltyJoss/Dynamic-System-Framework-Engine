#pragma once
// File:   Paths.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <string>
#include <filesystem>

namespace paths {
	/// Initialises the paths system
	ENGINE_API void init();
	// Accessors for the various paths used by the application
	ENGINE_API const std::filesystem::path& root();
	ENGINE_API const std::filesystem::path& assets();
	ENGINE_API const std::filesystem::path& configs();
	ENGINE_API const std::filesystem::path& logs();
	ENGINE_API const std::filesystem::path& runs();
} // namespace paths