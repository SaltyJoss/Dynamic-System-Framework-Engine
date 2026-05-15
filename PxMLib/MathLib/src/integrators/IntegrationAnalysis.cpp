#include "pch.h"
#include "integrators/IntegrationAnalysis.h"
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>

// Integration analysis methods
namespace integration {
	// Compute error statistics from a collection of error values
	ErrorStats analysis::computeErrorStats(const std::vector<double>& errors) {
		ErrorStats stats;
		if (errors.empty()) {
			stats.maxError = 0.0;
			stats.minError = 0.0;
			stats.meanError = 0.0;
			stats.rmsError = 0.0;
			return stats;
		}

		stats.maxError = -std::numeric_limits<double>::infinity();
		stats.minError = std::numeric_limits<double>::infinity();

		double sumError = 0.0;
		double sumSquaredError = 0.0;

		for (double error : errors) {
			if (error > stats.maxError) stats.maxError = error;
			if (error < stats.minError) stats.minError = error;
			sumError += error;
			sumSquaredError += error * error;
		}

		stats.meanError = sumError / errors.size();
		stats.rmsError = std::sqrt(sumSquaredError / errors.size());

		return stats;
	}

	// Compute error statistics from a collection of error samples
	void analysis::computeErrorStatsfromSamples(const std::vector<ErrorSample>& samples, ErrorStats& eulerStats, ErrorStats& midpointStats, ErrorStats& heunStats, ErrorStats& ralstonStats, ErrorStats& rk4Stats) {
		std::vector<double> eulerErrors;
		std::vector<double> midpointErrors;
		std::vector<double> heunErrors;
		std::vector<double> ralstonErrors;
		std::vector<double> rk4Errors;

		for (const auto& sample : samples) {
			eulerErrors.push_back(sample.errorEuler);
			midpointErrors.push_back(sample.errorMidpoint);
			heunErrors.push_back(sample.errorHeun);
			ralstonErrors.push_back(sample.errorRalston);
			rk4Errors.push_back(sample.errorRK4);
		}

		eulerStats = computeErrorStats(eulerErrors);
		midpointStats = computeErrorStats(midpointErrors);
		heunStats = computeErrorStats(heunErrors);
		ralstonStats = computeErrorStats(ralstonErrors);
		rk4Stats = computeErrorStats(rk4Errors);
	}

	// CSV export of error samples to file
	void analysis::exportErrorSamplesToCSV(const std::vector<ErrorSample>& samples, const std::string& filename) {
		std::ofstream file(filename);
		if (!file.is_open()) {
			throw std::runtime_error("Failed to open file for writing: " + filename);
		}

		// Write CSV header
		file << "Time,Error_Euler,Error_Midpoint,Error_Heun,Error_Ralston,Error_RK4\n";
		file << std::fixed << std::setprecision(10);

		// Write error samples
		for (const auto& sample : samples) {
			file << sample.time << ","
				<< sample.errorEuler << ","
				<< sample.errorMidpoint << ","
				<< sample.errorHeun << ","
				<< sample.errorRalston << ","
				<< sample.errorRK4 << "\n";
		}
		file.close();
	}
}