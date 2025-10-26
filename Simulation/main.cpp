#include <Eigen/Dense>
#include <iostream>
#include "MathLib/NumericalIntegrators.hpp"

using namespace Eigen;
using namespace MathLib::NumericalIntegrators::ODE;

int main() {
    VectorXd v(2);
    v << 1.0, 0.0;
	double t = 0.0;
	double dt = 0.1;

	// Simple harmonic oscillator: dx/dt = v, dv/dt = -x
	auto f = [](double t, const VectorXd &v) -> VectorXd {
		VectorXd dv(2);
		dv << v(1.0), -v(0.0);
		return dv;
	};

	v = rk4Step(v, t, dt, f);
	std::cout << "Next state: " << v.transpose() << std::endl;
	return 0;
}