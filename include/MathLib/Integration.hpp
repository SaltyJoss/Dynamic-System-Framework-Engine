#include <Eigen3/Dense>
#include <iostream>

using Eigen::VectorXd;

namespace mathlib::Integration {
	VectorXd integrate_ODE(
		const std::function<double(double, const double&)>& f,
		T y0, double t0, double t_final, double dt,
		const std::function<double(const std::function <double(double, const double&)>&, double, const double&, double)>& step_function
	) {
		// Applies logic from cpp file (see git commit changes)
		VectorXd trajectory;
		T y = y0;
		double t = t0;
		trajectory.push_back(y);

		while (t < t_final) {
			y = step_function(f, t, y, dt);
			t += dt;
			trajectory.push_back(y);
		}

		return trajectory;
	}
}