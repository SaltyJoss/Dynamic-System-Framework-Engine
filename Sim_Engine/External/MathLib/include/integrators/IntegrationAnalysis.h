#pragma once

#include "MathLibAPI.h"
#include <vector>
#include <string>

namespace integration {
	struct MATHLIB_API ErrorStats {
		double maxError;	// Maximum error observed
		double minError;	// Minimum error observed
		double meanError;	// Mean (average) error
		double rmsError;	// Root Mean Square (RMS) error
	};

	struct MATHLIB_API ErrorSample {
		double time;			// Time at which the error is sampled
		double errorEuler;		// Error for Euler method
		double errorMidpoint;	// Error for Midpoint method
		double errorHeun;		// Error for Heun's method
		double errorRalston;	// Error for Ralston's method
		double errorRK4;		// Error for Fourth-order Runge-Kutta method
	};

	class MATHLIB_API analysis {
	public:
		// Compute error statistics from a collection of error values
		ErrorStats computeErrorStats(const std::vector<double>& errors);

		// Compute error statistics from a collection of error samples
		void computeErrorStatsfromSamples(const std::vector<ErrorSample>& samples, ErrorStats& eulerStats, ErrorStats& midpointStats, ErrorStats& heunStats, ErrorStats& ralstonStats, ErrorStats& rk4Stats);

		// csv export of error samples to file
		void exportErrorSamplesToCSV(const std::vector<ErrorSample>& samples, const std::string& filename);
	};
}