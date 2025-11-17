#include <Eigen/Dense>
#include <functional>
using namespace Eigen;

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	class ODE_Integration {
	public:
		// Euler method
		VectorXd eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt);

		// Second-order Runge-Kutta method (Heun / Midpoint method)
		VectorXd rk2Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);

		// Fourth-order Runge-Kutta method
		VectorXd rk4Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);
	};

	// Partial Differential Equation (PDE) solvers
	class PDE_Integration {
	public:
		// Finite Difference Method (FDM) for discrete points in space
		VectorXd fdmStep(const VectorXd& u, double dx, double dt, double alpha);
	};
}