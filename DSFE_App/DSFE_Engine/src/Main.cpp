// DSFE_Engine Main.cpp
#include <EngineCore.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <Application.h>
#include "BatchEntry.h"
#include "BatchArgs.h"

// Helper function to split a comma-separated string into a vector of strings, trimming whitespace
static std::vector<std::string> splitComma(const std::string& input) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string item;
	// Split the input string by commas and trim whitespace from each item
    while (std::getline(ss, item, ','))
        if (!item.empty())
            result.push_back(item);

    return result;
}

// Helper function to parse a timestep (dt) from a string, supporting both plain decimal and fractional formats (e.g., "1/180")
static double parseDt(const std::string& input) {
    // Case: "1/180"
    auto slashPos = input.find('/');
    if (slashPos != std::string::npos) {
        double num = std::stod(input.substr(0, slashPos));
        double den = std::stod(input.substr(slashPos + 1));
        return num / den;
    }

    // Case: plain decimal
    return std::stod(input);
}

// Main entry point for the application, handling both GUI and batch modes based on command-line arguments
int main(int argc, char** argv) {
    bool batchMode = false;

	// First pass to check for batch mode flag
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--batch" || arg == "-b") {
            batchMode = true;
            break;
        }
    }

	// Non-batch mode: simple pass to handle GUI-specific options
    if (!batchMode) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if (arg == "--about") {
                std::cout
                    << "DSFE (Dynamic Systems Framework Engine)\n"
                    << " > A research-focused simulation engine for numerically modelling dynamic systems with different integration methods (explicit and implicit).\n"
                    << " > Version 0.7.2r-alpha\n"
                    << " > Developed by Joss Salton\n";
                return 0;
            }
            else if (arg == "--help" || arg == "-h") {
                std::cout << "Batch mode usage:\n";
                std::cout << "  --batch -t <script> [options]\n";
                std::cout << "Options:\n";
                std::cout << "  --basedt <value>       Baseline timestep (required)\n";
                std::cout << "  --baseint <method>    Baseline integrator (required)\n";
                std::cout << "  --dt <list>           Comma-separated list of timesteps to sweep\n";
                std::cout << "  --int <list>          Comma-separated list of integrators to sweep\n";
                std::cout << "  --name <value>        Run name for output organization\n";
                return 0;
            }
            else if (arg.starts_with("--")) {
                std::cerr << "Unknown option in GUI mode: " << arg << "\n";
				std::cout << "ERROR: Unknown option in GUI mode: " << arg << "\n";
                return 0;
            }
        }

        Application app("DSFE");
        return app.run();
    }

	// Batch mode: parse batch-specific arguments
    BatchArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--batch" || arg == "-b") { continue; }
        else if (arg == "--test" || arg == "-t") {
            if (i + 1 >= argc) {
                std::cerr << "--test requires a path.\n";
				std::cout << "ERROR: --test requires a path.\n";
                return EXIT_FAILURE;
            }
            args.testPath = argv[++i];
        }
        else if (arg == "--basedt") {
            if (i + 1 >= argc) {
                std::cerr << "--basedt requires value.\n";
				std::cout << "ERROR: --basedt requires value.\n";
                return EXIT_FAILURE;
            }
            args.baseDtRaw = argv[++i];
            args.baseDt = parseDt(args.baseDtRaw.value());
        }
        else if (arg == "--baseint") {
            if (i + 1 >= argc) {
                std::cerr << "--baseint requires value.\n";
				std::cout << "ERROR: --baseint requires value.\n";
                return EXIT_FAILURE;
            }
            args.baseInt = argv[++i];
        }
        else if (arg == "--dt") {
            if (i + 1 >= argc) {
                std::cerr << "--dt requires comma list.\n";
				std::cout << "ERROR: --dt requires comma list.\n";
                return EXIT_FAILURE;
            }
            auto list = splitComma(argv[++i]);
            for (auto& s : list) { 
                args.sweepDtRaw.push_back(s);
                args.sweepDt.push_back(parseDt(s));
            }
        }
        else if (arg == "--int") {
            if (i + 1 >= argc) {
                std::cerr << "--int requires comma list.\n";
				std::cout << "ERROR: --int requires comma list.\n";
                return EXIT_FAILURE;
            }
            args.sweepInt = splitComma(argv[++i]);
        }
        else if (arg == "--name") {
            if (i + 1 >= argc) {
                std::cerr << "--name requires value.\n";
				std::cout << "ERROR: --name requires value.\n";
                return EXIT_FAILURE;
            }
            args.runName = argv[++i];
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Batch mode usage:\n"
                << "  --batch -t <script> [options]\n"
                << "Options:\n"
                << "  --basedt <value>      Baseline timestep (required)\n"
                << "  --baseint <method>    Baseline integrator (required)\n"
                << "  --dt <list>           Comma-separated list of timesteps to sweep\n"
                << "  --int <list>          Comma-separated list of integrators to sweep\n"
                << "  --name <value>        Run name for output organization\n"
                << "----------\n"
                << " Supported Integrators:\n"
                << "  Explicit:\n"
                << "    > euler\n"
                << "    > midpoint\n"
                << "    > heun\n"
                << "    > ralston\n"
                << "    > rk4\n"
                << "    > rk45\n"
                << "  Implicit:\n"
                << "    > implicit_euler\n"
                << "    > implicit_midpoint\n"
                << "    > glrk2\n"
                << "    > glrk3\n"
                << "----------\n"
                << " Example:\n"
                << " Engine.exe --batch -t tests/balance_test.dsl --basedt 0.01 --baseint rk4 --dt 0.01,0.005,0.001 --int rk4,rk45 --name balance_sweep\n";
            return 0;
        }
        else {
            std::cerr << "Unknown batch argument: " << arg << "\n";
            std::cout << "ERROR: Unknown batch argument: " << arg << "\n";
            return EXIT_FAILURE;
        }
    }

	// Basic validation
    if (args.testPath.empty()) {
        std::cerr << "Batch mode requires --test <script>\n";
		std::cout << "ERROR: Batch mode requires --test <script>\n";
        return EXIT_FAILURE;
    }

	// Run batch mode with parsed arguments
    return runBatchMode(args);
}