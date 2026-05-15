#pragma once
#include <string>
#include <vector>
#include <optional>

// Struct to hold command-line arguments for batch mode
struct BatchArgs {
    std::string testPath;
	// Base configuration for batch runs (if not provided, defaults will be used)
    std::optional<std::string> baseDtRaw;
    std::optional<double> baseDt;
    std::optional<std::string> baseInt;
	// Lists for sweep parameters (if not provided, defaults will be used)
    std::vector<std::string> sweepDtRaw;
    std::vector<double> sweepDt;
    std::vector<std::string> sweepInt;
	// Run name for output organisation
    std::optional<std::string> runName;
};