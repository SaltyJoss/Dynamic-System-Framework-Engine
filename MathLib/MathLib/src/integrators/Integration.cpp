#include "pch.h"
#include "integrators/integration.h"

namespace integration {
	// General ODE integrator function
	VectorXd integration::integrate_ODE(
		const std::function<double(double, const double&)>& f,
		double y0, double t0, double t_final, double dt,
		const std::function<double(const std::function <double(double, const double&)>&, double, const double&, double)>& step_function
	) {
		VectorXd trajectory;
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
	VectorXd integration::integrate_PDE(
		const std::function<VectorXd(double, const VectorXd&)>& f,
		const VectorXd& u0, double t0, double t_final, double dt,
		const std::function<VectorXd(const std::function <VectorXd(double, const VectorXd&)>&, double, const VectorXd&, double)>& step_function
	) {
		std::vector<VectorXd> trajectory;
		VectorXd u = u0;
		double t = t0;
		trajectory.push_back(u);
		while (t < t_final) {
			u = step_function(f, t, u, dt);
			t += dt;
			trajectory.push_back(u);
		}
		// Convert std::vector to Eigen::VectorXd (flattened)
		int total_size = trajectory.size() * u0.size();
		VectorXd result(total_size);
		for (size_t i = 0; i < trajectory.size(); ++i) {
			result.segment(i * u0.size(), u0.size()) = trajectory[i];
		}
		return result;
	}

	inline void integration::append(Eigen::VectorXd& v, double value) {
		v.conservativeResize(v.size() + 1);
		v(v.size() - 1) = value;
	}
}