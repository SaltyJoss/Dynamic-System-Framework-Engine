#include "pch.h"
#include "control/Metrics.h"

namespace control {
	/// <inheritdoc/>
	double Metrics::rmse(const std::vector<VecX>& errors) {
		if (errors.empty()) return 0.0;
		double sum_sq = 0.0;
		size_t n = 0;
		for (const auto& err : errors) {
			sum_sq += err.squaredNorm();
			n += err.size();
		}
		return sqrt(sum_sq / n);
	}

	/// <inheritdoc/>
	double Metrics::maxNorm(const std::vector<VecX>& errors) {
		double max_norm = 0.0;
		for (const auto& err : errors) {
			double norm = err.norm();
			if (norm > max_norm) {
				max_norm = norm;
			}
		}
		return max_norm;
	}

	/// <inheritdoc/>
	double Metrics::settlingTime(const std::vector<VecX>& errors, double dt, double threshold) {
		for (size_t i = errors.size(); i-- > 0;) {
			if (errors[i].norm() > threshold) {
				return (i + 1) * dt; // Return time just after the last exceedance
			}
		}
		return 0.0; // System is always within threshold
	}
}