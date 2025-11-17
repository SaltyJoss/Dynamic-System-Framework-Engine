#include <Eigen/Dense>
#include <iostream>

using Eigen::VectorXd;

namespace integration {
	class integrator {
	public:
		// General ODE integrator function
		VectorXd integrate_ODE(
			const std::function<double(double, const double&)>& f,
			double y0, double t0, double t_final, double dt,
			const std::function<double(const std::function <double(double, const double&)>&, double, const double&, double)>& step_function
		);

		// General PDE integrator function
		VectorXd integrate_PDE(
			const std::function<VectorXd(double, const VectorXd&)>& f,
			const VectorXd& u0, double t0, double t_final, double dt,
			const std::function<VectorXd(const std::function <VectorXd(double, const VectorXd&)>&, double, const VectorXd&, double)>& step_function
		);

		// Helper function to append a value to an Eigen::VectorXd (originally tried to use push_back, but Eigen doesn't support it)
		inline void append(Eigen::VectorXd& v, double value);
	};
}