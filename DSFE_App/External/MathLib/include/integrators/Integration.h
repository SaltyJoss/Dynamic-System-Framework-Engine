#pragma once
#pragma warning(disable : 4251)

#include "MathLibAPI.h"

#include "core/Types.h"
#include <functional>

using namespace mathlib;

namespace integration {
	class MATHLIB_API integrator {
	public:
		// General ODE integrator function
		VecX integrate_ODE(
			const std::function<double(double, const double&)>& f,
			double y0, double t0, double t_final, double dt,
			const std::function<double(const std::function <double(double, const double&)>&, double, const double&, double)>& step_function
		);

		// General PDE integrator function
		VecX integrate_PDE(
			const std::function<VecX(double, const VecX&)>& f,
			const VecX& u0, double t0, double t_final, double dt,
			const std::function<VecX(const std::function <VecX(double, const VecX&)>&, double, const VecX&, double)>& step_function
		);

	private: 
		// Helper function to append a value to an Eigen::VecX (originally tried to use push_back, but Eigen doesn't support it)
		static void append(VecX& v, double value);
	};
}