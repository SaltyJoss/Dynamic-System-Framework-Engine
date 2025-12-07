#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

namespace control {
	class MATHLIB_API Metrics {
	public:
		/// <summary>
		/// Calculates the Root Mean Square Error (RMSE) of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The RMSE value.</returns>
		double rmse(const std::vector<VecX>& errors);

		/// <summary>
		/// Calculates the maximum norm of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The maximum norm value.</returns>
		double maxNorm(const std::vector<VecX>& errors);

		/// <summary>
		/// Calculates the settling time of a system based on the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <param name="dt">The time step between each error measurement.</param>
		/// <param name="threshold">The error threshold to consider the system settled.</param>
		/// <returns>The settling time in seconds.</returns>
		double settlingTime(const std::vector<VecX>& errors, double dt, double threshold);
	};
}