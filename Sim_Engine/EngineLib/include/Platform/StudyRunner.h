#pragma once
// File:   StudyRunner.h
// GitHub: SaltyJoss
#include <vector>
#include <string>
#include <future>
#include <functional>
#include <memory>
#include <atomic>

#include "Platform/ISimulationCore.h"
#include "Interpreter/StoredProgram.h"

// Type alias for a unique_ptr to ISimulationCore with a custom deleter (using std::function for flexibility)
using CorePtr = std::unique_ptr<core::ISimulationCore, std::function<void(core::ISimulationCore*)>>;

// Struct to hold the results of a study/research run
struct ENGINE_API StudyResult {
	bool success = false; // whether the run completed successfully
	std::string tag = "N/A"; // user-provided tag for the run
	std::string intName = "N/A"; // name of the integrator used
	double dt = 0.0;
	double simTime = 0.0;
	size_t samples = 0;
};

// StudyRunner class to manage running batches of studies in parallel
class ENGINE_API StudyRunner {
public:
	// Config for a run
	struct config {
		integration::eIntegrationMethod method;
		double dt = 0.0;		// timestep (secs)
		double len_min = 0.0;	// length of run (mins)
		std::string tag;		// user-provided id for run
	};

	// Factory that creates a full configured SimulationCore for a run (its a fresh instance)
	using MakeCoreFn = std::function<CorePtr(void)>;

	// Constructor and destructor
	StudyRunner(MakeCoreFn makeCore, size_t maxConcurrency = 0);
	~StudyRunner() = default;

	// Run a batch of studies in parallel using the script string as the "run" for each sim, returning the results when all are complete
	std::vector<StudyResult> runStudies(const std::vector<config>& configs, const std::string& scriptText);

private:
	MakeCoreFn _makeCore;	// Factory function to create SimulationCore instances for each run
	size_t _maxConcurrency; // Maximum number of concurrent runs (0 for hardware concurrency - limit)
};

// Keeping out of a namespace so I can avoid avoid circular dependencies, also so it is easily accessed without many nested namespaces :)