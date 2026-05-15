#pragma once
#pragma warning(disable : 4251)

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace control {
	class MATHLIB_API Metrics {
	public:
		/// <summary>
		/// Calculates the Mean Squared Error (MSE) of the provided error signals.
		/// </summary>
		/// <param name="errors">The errors.</param>
		/// <returns>The MSE value.</returns>
		double MSE(const std::vector<VecX>& errors) {
			if (errors.empty()) return 0.0;
			double sum_sq = 0.0;
			size_t n = 0;
			for (const auto& err : errors) {
				sum_sq += err.squaredNorm();
				n += err.size();
			}
			return sum_sq / n;
		}

		/// <summary>
		/// Calculates the Root Mean Square Error (RMSE) of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The RMSE value.</returns>
		double RMSE(const std::vector<VecX>& errors) {
			return sqrt(MSE(errors));
		}

		/// <summary>
		/// Calculates the maximum norm of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The maximum norm value.</returns>
		double maxNorm(const std::vector<VecX>& errors) {
			double max_norm = 0.0;
			for (const auto& err : errors) {
				double norm = err.norm();
				if (norm > max_norm) {
					max_norm = norm;
				}
			}
			return max_norm;
		}

		/// <summary>
		/// Calculates the settling time of a system based on the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <param name="dt">The time step between each error measurement.</param>
		/// <param name="threshold">The error threshold to consider the system settled.</param>
		/// <returns>The settling time in seconds.</returns>
		double settlingTime(const std::vector<VecX>& errors, double dt, double threshold) {
			for (size_t i = errors.size(); i-- > 0;) {
				if (errors[i].norm() > threshold) {
					return (i + 1) * dt; // Return time just after the last exceedance
				}
			}
			return 0.0; // System is always within threshold
		}
	};
}