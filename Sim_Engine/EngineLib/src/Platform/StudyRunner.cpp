#include "pch.h"
// File:   StudyRunner.cpp
// GitHub: SaltyJoss
#include "Platform/StudyRunner.h"
#include "Interpreter/Parser.h"
#include <thread>
#include <chrono>
#include <iostream>

/* Fair warning:
 * ------------- 
 * I am going to be making ALOT of notes here as I am covering new ground for with C++ concurrency and async programming.
 * This way I can understand it better and actually reference it later without having to re-learn and re-read a bunch of documentation and examples.
 * I will try reduce it incase anyone is using this code, or reading through it.
 * That being said, I am a student first, and my primary purpose is to learn about these subjects.
 * I cannot be bothered programming all of this to just not understand it - wasted time, and wasted opportunity honestly.
 */

// Constructor
StudyRunner::StudyRunner(MakeCoreFn makeCore, size_t maxConcurrency)
	: _makeCore(std::move(makeCore)) {
	if (!_makeCore) { throw std::invalid_argument("StudyRunner constructor got null makeCore function"); }

	// If maxConcurrency is 0, use hardware concurrency (number of CPU cores) as the limit for concurrent runs
	if (maxConcurrency == 0) {
		unsigned int hc = std::thread::hardware_concurrency();
		_maxConcurrency = (hc == 0 ? 2u : hc); // Fallback if hardware_concurrency returns 0 (not well-defined or undetectable)
		// NOTE: 2u means unsigned int literal with value 2 (I genuinely forgot the syntax for unsigned literals and had to look it up)
	}
	else { _maxConcurrency = maxConcurrency; }
}

// Helper function to find the index of the first future that is ready in a vector of futures
// NOTE: future is a synchronisation primitive that represents a val which will be available at some point in the future (logged for my own re-readability)
static int findReadyIndex(std::vector<std::future<StudyResult>>& futures) {
	// Check each future for readiness without blocking, returning the index of the first ready future
	for (size_t i = 0; i < futures.size(); ++i) {
		// Check if this future is ready by waiting for 100ms (non-blocking)
		if (futures[i].wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) { return (int)i; }
	}
	return -1; // No future is ready yet
}

// Run a batch of studies in parallel using the script string as the "run" for each sim, returning the results when all are complete
std::vector<StudyResult> StudyRunner::runStudies(const std::vector<config>& configs, std::string& scriptText) {
	std::vector<std::future<StudyResult>> futures; // Vector to hold the futures for each async run
	futures.reserve(configs.size());
	
	for (size_t i = 0; i < configs.size(); ++i) {
		const config& cfg = configs[i];

		// Before starting a new async run, check if we have reached the max concurrency limit.
		// If we have, we need to wait for at least one of the existing runs to finish before starting a new one.
		while (futures.size() >= _maxConcurrency) {
			int idx = findReadyIndex(futures); // Check for a ready future without blocking the main thread
			if (idx >= 0) {
				// collect results from ready future, and remove it from the vector
				try {
					StudyResult r = futures[idx].get(); // get() will block if the future is not ready
					(void)r; // silence unused variable warning
				}
				// catch any exceptions thrown during the async run and log them, but continue processing other runs (very important to my initial implementation goal!!)
				catch (const std::exception& e) {
					std::cerr << "Error in study run: " << e.what() << std::endl;
				}
				futures.erase(futures.begin() + idx); // remove the future from the vector after collecting its result
			}
			else { std::this_thread::sleep_for(std::chrono::milliseconds(5)); } // Sleep briefly to avoid busy-waiting if no futures are ready
		}
		// Start a new async run for this config, capturing the current config and program by value to make sure they are safely used in the async context
		futures.push_back(std::async(std::launch::async, [this, cfg, scriptText]() -> StudyResult {
			StudyResult result{};
			result.tag = cfg.tag;
			result.dt = cfg.dt;
			
			auto simCore = _makeCore();
			// If we couldn't create a SimulationCore, return a failed result immediately
			if (!simCore) {
				result.success = false;
				result.intName = "N/A";
				result.simTime = 0.0;
				result.samples = 0;
				return result;
			}
			// Configure the SimulationCore for this run before building the program
			simCore->setFixedDt(cfg.dt);
			simCore->setIntegrationMethod(cfg.method);

			// Create a program and parser for this run, bound to the SimulationCore we just created
			auto program = std::make_unique<interpreter::StoredProgram>(simCore.get());
			interpreter::Parser parser(program.get());
			// Parse the script text to build the program for this run, and start it
			parser.parse(scriptText);
			program->start();

			// Run synchronously (blocking inside thread)
			bool ok = simCore->runScriptToCompletion(program.get(), cfg.method); // Run the provided program/script to completion with method, blocking until it finishes.
			// NOTE: This is where the actual study run happens, it is block because it is inside the async thread, so it will not block the main thread (OR other runs), and allows DSFE to have multiple runs in parallel!!! <- EXACTLY what I want

			// Get telemetry data for results
			result.success = ok;
			result.intName = simCore->integrationMethodName(); // Get the name of the current integration method for reporting
			result.simTime = simCore->simTime(); // Get the total simulation time that elapsed during this run
			// Telemetry sample count is implementation-defined -> guarded for availability
			try {
				result.samples = simCore->telemetrySampleCount();
			}
			// If telemetry is not available or throws an exception, it is caught and samples to 0 to indicate no data was collected
			catch (...) {
				result.samples = 0;
			}
			// Get the number of telemetry samples collected during this run (proxy for how much data we obtained)
			return result; // Return the result of this study run
		}));
	}

	// Collect the results from all the futures
	std::vector<StudyResult> results; // Vector to hold the final results of all runs
	for (auto& f : futures) {
		// Wait for each future to be ready and collect its result
		try {
			StudyResult r = f.get(); // get() will block until the future is ready, collecting the result of each run
			results.push_back(std::move(r)); // Move the result into the results vector to avoid unnecessary copying
		}
		// catch any exceptions thrown during the async runs and log them
		catch (const std::exception& e) {
			std::cerr << "Error collecting study result: " << e.what() << std::endl;
			// Create a failed result to represent the exception case
			StudyResult bad;
			bad.success = false;
			bad.tag = "exception";
			// Push a failed/bad result
			results.push_back(bad);
		}
	}
	return results; // Return the collected results of all study runs
}
	