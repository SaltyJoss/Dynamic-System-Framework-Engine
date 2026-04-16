// BatchMain.cpp
#include "pch.h"
#include <EngineCore.h>
#include "BatchEntry.h"
#include "BatchArgs.h"
#include "Platform/StudyRunner.h"
#include "Numerics/IntegrationMethods.h"
#include "Platform/Paths.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

// Integration method parser from string (throws if unknown)
static integration::eIntegrationMethod parseMethod(const std::string& name) {
    // Normalise input: lower-case, convert spaces and dashes to underscores
    std::string s;
    s.reserve(name.size());
    for (unsigned char c : name) {
        if (c == ' ' || c == '-') s.push_back('_');
        else s.push_back(static_cast<char>(std::tolower(c)));
    }

	// Explicit
    if (s == "euler")    return integration::eIntegrationMethod::Euler;
    if (s == "midpoint") return integration::eIntegrationMethod::Midpoint;
    if (s == "heun")     return integration::eIntegrationMethod::Heun;
    if (s == "ralston")  return integration::eIntegrationMethod::Ralston;
    if (s == "rk4")      return integration::eIntegrationMethod::RK4;
    if (s == "rk45")     return integration::eIntegrationMethod::RK45;
    // Implicit
    if (s == "implicit_euler")    return integration::eIntegrationMethod::ImplicitEuler;
    if (s == "implicit_midpoint") return integration::eIntegrationMethod::ImplicitMidpoint;
	if (s == "glrk2")             return integration::eIntegrationMethod::GLRK2;
	if (s == "glrk3")             return integration::eIntegrationMethod::GLRK3;
    throw std::runtime_error("Unknown integrator: " + name);
}

// Helper to format a timestep (dt) as a string for use in tags and filenames
static std::string formatDtForFile(const std::string& raw) {
    std::string out;
    out.reserve(raw.size() + 4);
    for (char c : raw) {
        if (c == '.') { out += "p"; }
        else if (c == '/') { out += "over"; }
        else { out += c; }
    }
    return out;
}

// Core factory function for StudyRunner
static CorePtr makeCoreFactory() {
    core::ISimulationCore* raw = CreateSimulationCore_v1();
    if (!raw) { return CorePtr(nullptr, [](core::ISimulationCore*) {}); }
    return CorePtr(raw, [](core::ISimulationCore* p) { DestroySimulationCore(p); });
}

// Batch mode entry point
int runBatchMode(const BatchArgs& args) {
    paths::init();
    fprintf(stdout, "BATCH MODE ENTERED\n");
    fflush(stdout);
	// Basic validation and setup
    try {
        // Validate required baseline
        if (!args.baseDt.has_value() || !args.baseInt.has_value()) { throw std::runtime_error("Baseline (--basedt and --baseint) required."); }
        if (args.testPath.empty()) { throw std::runtime_error("No test script provided."); }

        // Load script
        std::ifstream file(args.testPath);
        if (!file) { throw std::runtime_error("Failed to open test file: " + args.testPath); }
        std::ostringstream buffer;
        buffer << file.rdbuf();
        std::string scriptText = buffer.str();

        // Build configs
        std::vector<StudyRunner::config> configs;

        // Baseline (always first)
        {
            StudyRunner::config base;
            base.method = parseMethod(args.baseInt.value());
            base.dt = args.baseDt.value();
            base.len_min = 1.0;   // Fixed simulation duration (consistent across runs)
            base.tag =
                (args.runName.has_value() ? args.runName.value() + "_" : "") +
                "base_dt" + formatDtForFile(args.baseDtRaw.value()) + 
                "_int_" + args.baseInt.value();
            configs.push_back(base);
        }

        // Sweep grid
        for (size_t i = 0; i < args.sweepDt.size(); ++i) {
            double dt = args.sweepDt[i];
            const std::string& dtRaw = args.sweepDtRaw[i];
            for (const auto& intName : args.sweepInt) {
                StudyRunner::config c;
                c.method = parseMethod(intName);
                c.dt = dt;
                c.len_min = 1.0;
                c.tag =
                    (args.runName.has_value() ? args.runName.value() + "_" : "") +
                    "dt" + formatDtForFile(dtRaw) +
                    "_int_" + intName;
                configs.push_back(c);
            }
        }

        // Worker configuration
        size_t workers = 1; // stable mode (parallel logging disabled)
        StudyRunner runner(makeCoreFactory, workers);

        // Execute
        auto T0 = std::chrono::high_resolution_clock::now();
        auto results = runner.runStudies(configs, scriptText);
        auto T1 = std::chrono::high_resolution_clock::now();

        double total = std::chrono::duration<double>(T1 - T0).count();
        fprintf(stderr, "TOTAL BATCH TIME = %.3f sec\n", total);

        // Sort results by tag
        std::sort(results.begin(), results.end(),
            [](const StudyResult& a, const StudyResult& b) {
                return a.tag < b.tag;
            });

        // Print summary
        for (const auto& r : results) {
            std::cout
                << "tag=" << r.tag
                << " success=" << (r.success ? "yes" : "no")
                << " int=" << r.intName
                << " time=" << r.simTime
                << " samples=" << r.samples
                << "\n";
        }
    }
	// Catch any exceptions that escaped from the batch processing and log them
    catch (const std::exception& e) {
        std::cerr << "Batch mode exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}