// BatchMain.cpp
#include "pch.h"
#include <EngineCore.h>
#include "include/BatchEntry.h"
#include "Platform/StudyRunner.h"
#include "Numerics/IntegrationMethods.h"

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

        // Build StudyRunner with factory
        StudyRunner runner([]() { return makeCoreFactory(); }, /*maxConcurrency=*/ 0);

        // The script text for the run(s)
        std::string scriptText = R"(
            // your DSL script here, example:
            set(integrator, rk4)
            // ... more script ...
        )";

        // Run studies (note: runStudies signature expects std::string& in your header)
        auto results = runner.runStudies(configs, scriptText);

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