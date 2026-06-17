// DSFE_Core Paths.h
#pragma once
#include "EngineCore.h"
#include <string>
#include <filesystem>

namespace paths {
	/// Initialises the paths system
	DSFE_API void init();
	// Accessors for the various paths used by the application
	DSFE_API const std::filesystem::path& root();
	DSFE_API const std::filesystem::path& assets();
	DSFE_API const std::filesystem::path& configs();
	DSFE_API const std::filesystem::path& logs();
	DSFE_API const std::filesystem::path& runs();
} // namespace paths