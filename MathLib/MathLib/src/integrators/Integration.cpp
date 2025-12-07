#include "pch.h"
#include "integrators/Integration.h"


namespace integration {
	// General ODE integrator function
	VecX integrator::integrate_ODE(
		const std::function<double(double, const double&)>& f,
		double y0, double t0, double t_final, double dt,
		const std::function<double(const std::function <double(double, const double&)>&, double, const double&, double)>& step_function
	) {
		VecX trajectory;
		double y = y0;
		double t = t0;
		append(trajectory, y);

		while (t < t_final) {
			y = step_function(f, t, y, dt);
			t += dt;
			append(trajectory, y);
		}

		return trajectory;
	}

	// General PDE integrator function
	VecX integrator::integrate_PDE(
		const std::function<VecX(double, const VecX&)>& f,
		const VecX& u0, double t0, double t_final, double dt,
		const std::function<VecX(const std::function <VecX(double, const VecX&)>&, double, const VecX&, double)>& step_function
	) {
		std::vector<VecX> trajectory;
		VecX u = u0;
		double t = t0;
		trajectory.push_back(u);
		while (t < t_final) {
			u = step_function(f, t, u, dt);
			t += dt;
			trajectory.push_back(u);
		}
		// Convert std::vector to Eigen::VecX (flattened)
		int total_size = trajectory.size() * u0.size();
		VecX result(total_size);
		for (size_t i = 0; i < trajectory.size(); ++i) {
			result.segment(i * u0.size(), u0.size()) = trajectory[i];
		}
		return result;
	}

	void integrator::append(VecX& v, double value) {
		v.conservativeResize(v.size() + 1);
		v(v.size() - 1) = value;
	}
}