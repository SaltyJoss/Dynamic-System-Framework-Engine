// BatchMain.cpp
#include "pch.h"
#include <EngineCore.h>
#include "include/BatchEntry.h"
#include "Platform/StudyRunner.h"
#include "Numerics/IntegrationMethods.h"

#include <chrono>   // ensure at top of file
#include <cstdio>   // for fprintf


// Helper: create CorePtr (unique_ptr with std::function deleter)
static CorePtr makeCoreFactory() {
    core::ISimulationCore* raw = CreateSimulationCore_v1();
    if (!raw) {
        return CorePtr(nullptr, [](core::ISimulationCore*) {});
    }
    // std::function deleter is constructed from the lambda implicitly
    return CorePtr(raw, [](core::ISimulationCore* p) { DestroySimulationCore(p); });
}

int runBatchMode() {
    fprintf(stdout, "BATCH MODE ENTERED\n");
    fflush(stdout);
    try {
		// Define the configurations for the studies to run (combinations of integrator methods, timesteps, and run lengths)
        std::vector<StudyRunner::config> configs;
        std::vector<integration::eIntegrationMethod> methods = {
            integration::eIntegrationMethod::Euler,
            integration::eIntegrationMethod::Midpoint,
            integration::eIntegrationMethod::Heun,
            integration::eIntegrationMethod::Ralston,
            integration::eIntegrationMethod::RK4,
            integration::eIntegrationMethod::RK45
        };
        std::vector<double> dts = { 0.001, 0.002, 0.005 };
        std::vector<double> lengths_mins = { 0.5, 1.0, 5.0 };
		// Create a config for each combination of method, dt, and length
        for (auto m : methods) {
            for (double dt : dts) {
                for (double len : lengths_mins) {
                    StudyRunner::config c;
                    c.method = m;
                    c.dt = dt;
                    c.len_min = len;
                    c.tag = std::string("m") + std::to_string((int)m) + "_dt" + std::to_string(dt) + "_L" + std::to_string((int)len);
                    configs.push_back(c);
                }
            }
        }

		// Determine the number of worker threads to use based on hardware concurrency
        size_t cores = std::thread::hardware_concurrency();
        size_t workers = 0; // default: use all cores

		// If not in batch mode only, reserve one core for the main thread
#ifndef _BATCH_MODE_ONLY
        workers = (cores > 1) ? cores - 1 : 1;
#endif
		// Build runner with factory and worker count
        StudyRunner runner(makeCoreFactory, workers);

        // The script text for the run(s)
        std::string scriptText = R"(
            set(integrator, rk4)
            start()
        )";

		// Run the studies and time the total batch duration
        auto T0 = std::chrono::high_resolution_clock::now();
        auto results = runner.runStudies(configs, scriptText);
        auto T1 = std::chrono::high_resolution_clock::now();

		// Log total batch time
        double total = std::chrono::duration<double>(T1 - T0).count();
        fprintf(stderr, "TOTAL BATCH TIME = %.3f sec\n", total);

		// Sort results by tag for easier reading
        std::sort(results.begin(), results.end(),
            [](const StudyResult& a, const StudyResult& b) {
                return a.tag < b.tag;
            });

        // Print summary
        for (const auto& r : results) {
            std::cout << "tag=" << r.tag
                << " success=" << (r.success ? "yes" : "no")
                << " int=" << r.intName
                << " time=" << r.simTime
                << " samples=" << r.samples << "\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Batch mode exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}