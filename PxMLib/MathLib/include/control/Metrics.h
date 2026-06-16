// PxM/MathLib Metrics.h
#pragma once
#pragma warning(disable : 4251)

#include <core/MathLib.h>

using namespace mathlib;

namespace control {
	class Metrics {
	public:
		/// <summary>
		/// Calculates the Mean Squared Error (MSE) of the provided error signals.
		/// </summary>
		/// <param name="errors">The errors.</param>
		/// <returns>The MSE value.</returns>
		template<typename Scalar>
		inline Scalar MSE(const std::vector<VecX_T<Scalar>>& errors) {
			if (errors.empty()) { return Scalar(0); }
			Scalar sum_sq = Scalar(0);
			size_t n = Scalar(0);
			for (const auto& err : errors) {
				sum_sq += norm2(err);
				n += (size_t)err.size();
			}
			return sum_sq / n;
		}

		/// <summary>
		/// Calculates the Root Mean Square Error (RMSE) of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The RMSE value.</returncs>
		template<typename Scalar>
		inline Scalar RMSE(const std::vector<VecX_T<Scalar>>& errors) { return sqrt(MSE(errors)); }

		/// <summary>
		/// Calculates the maximum norm of the provided error signals.
		/// </summary>
		/// <param name="errors">A vector of error vectors over time.</param>
		/// <returns>The maximum norm value.</returns>
		template<typename Scalar>
		Scalar maxNorm(const std::vector<VecX_T<Scalar>>& errors) {
			Scalar max_norm = Scalar(0);
			for (const auto& err : errors) {
				Scalar norm = norm(err);
				max_norm = max(max_norm, norm);
			}
			return max_norm;
		}
	};
}