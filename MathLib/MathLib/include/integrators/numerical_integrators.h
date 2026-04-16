#pragma once
// File:   numerical_integrators.h
// GitHub: SaltyJoss
#include "MathLibAPI.h"

#include "core/Types.h"
#include <string>
#include <functional>
#include <iostream>

using namespace mathlib;

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	class MATHLIB_API ODE {
	public:
		// Euler method
		template<typename Func>
		inline VecX eulerStep(const VecX& x, double t, double dt, Func&& f) {
			return x + dt * f(t, x);
		}

		// Second-order Runge-Kutta method (Midpoint)
		template<typename Func>
		inline VecX midpointStep(const VecX& x, double t, double dt, Func&& f) {
			VecX k1 = dt * f(t, x);
			VecX k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
			return x + k2;
		}

		// Second-order Runge-Kutta method (Heun)
		template<typename Func>
		inline VecX heunStep(const VecX& x, double t, double dt, Func&& f) {
			VecX k1 = dt * f(t, x);
			VecX k2 = dt * f(t + dt, x + k1);
			return x + (k1 + k2) / 2.0;
		}

		// Second-order Runge-Kutta method (Ralston)
		template<typename Func>
		inline VecX ralstonStep(const VecX& x, double t, double dt, Func&& f) {
			VecX k1 = dt * f(t, x);
			VecX k2 = dt * f(t + (2.0 / 3.0) * dt, x + (2.0 / 3.0) * k1);
			return x + (k1 + 3.0 * k2) / 4.0;
		}

		// Fourth-order Runge-Kutta method
		template<typename Func>
		inline VecX rk4Step(const VecX& x, double t, double dt, Func&& f) {
			VecX k1 = dt * f(t, x);
			VecX k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
			VecX k3 = dt * f(t + dt / 2.0, x + k2 / 2.0);
			VecX k4 = dt * f(t + dt, x + k3);
			return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
		}

		// RK45 method with adaptive step size (Dormand-Prince)
		template<typename Func>
		inline VecX rk45Step(const VecX& x, double t, double& dt, double& dt_used, Func&& f, double rtol, double atol) {
			const double safety = 0.9;	// safety factor to prevent aggressive step size changes
			const double fac_min = 0.2;	// minimum factor for reducing step size
			const double fac_max = 5.0;	// maximum factor for increasing step size
			const double h_min = 1e-10;	// minimum allowed step size
			const double h_max = 1.0;	// maximum allowed step size

			dt = std::clamp(dt, h_min, h_max);

			// Try up to 25 attempts to find an acceptable step size
			for (int attempt = 0; attempt < 25; ++attempt) {
				// Butcher tableau for Dormand-Prince method

				// Coefficients for error estimation
				const double c2 = 1.0 / 5.0;
				const double c3 = 3.0 / 10.0;
				const double c4 = 4.0 / 5.0;
				const double c5 = 8.0 / 9.0;
				const double c6 = 1.0;
				const double c7 = 1.0;

				// Dormand-Prince coefficients
				const double a21 = 1.0 / 5.0;
				const double a31 = 3.0 / 40.0;
				const double a32 = 9.0 / 40.0;
				const double a41 = 44.0 / 45.0;
				const double a42 = -56.0 / 15.0;
				const double a43 = 32.0 / 9.0;
				const double a51 = 19372.0 / 6561.0;
				const double a52 = -25360.0 / 2187.0;
				const double a53 = 64448.0 / 6561.0;
				const double a54 = -212.0 / 729.0;
				const double a61 = 9017.0 / 3168.0;
				const double a62 = -355.0 / 33.0;
				const double a63 = 46732.0 / 5247.0;
				const double a64 = 49.0 / 176.0;
				const double a65 = -5103.0 / 18656.0;
				const double a71 = 35.0 / 384.0;
				const double a72 = 0.0;
				const double a73 = 500.0 / 1113.0;
				const double a74 = 125.0 / 192.0;
				const double a75 = -2187.0 / 6784.0;
				const double a76 = 11.0 / 84.0;

				// Weights for 4th and 5th order estimates
				const double b1 = 35.0 / 384.0;
				const double b2 = 0.0;
				const double b3 = 500.0 / 1113.0;
				const double b4 = 125.0 / 192.0;
				const double b5 = -2187.0 / 6784.0;
				const double b6 = 11.0 / 84.0;
				const double b1s = 5179.0 / 57600.0;
				const double b2s = 0.0;
				const double b3s = 7571.0 / 16695.0;
				const double b4s = 393.0 / 640.0;
				const double b5s = -92097.0 / 339200.0;
				const double b6s = 187.0 / 2100.0;
				const double b7s = 1.0 / 40.0;

				// Compute the Runge-Kutta stages (DP -> 7 stages)
				const VecX k1 = f(t, x);
				const VecX k2 = f(t + c2 * dt, x + dt * (a21 * k1));
				const VecX k3 = f(t + c3 * dt, x + dt * (a31 * k1 + a32 * k2));
				const VecX k4 = f(t + c4 * dt, x + dt * (a41 * k1 + a42 * k2 + a43 * k3));
				const VecX k5 = f(t + c5 * dt, x + dt * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4));
				const VecX k6 = f(t + c6 * dt, x + dt * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5));
				const VecX k7 = f(t + c7 * dt, x + dt * (a71 * k1 + a72 * k2 + a73 * k3 + a74 * k4 + a75 * k5 + a76 * k6));

				// Compute 4th and 5th order estimates
				const VecX y5 = x + dt * ((b1 * k1) + (b3 * k3) + (b4 * k4) + (b5 * k5) + (b6 * k6));   // 5th order estimate
				const VecX y4 = x + dt * ((b1s * k1) + (b3s * k3) + (b4s * k4) + (b5s * k5) + (b6s * k6) + (b7s * k7)); // 4th order estimate

				const VecX e = y5 - y4; // Error estimate

				// Compute the error norm
				double errNorm = 0.0;
				for (int i = 0; i < e.size(); ++i) {
					double sc = atol + rtol * std::max<double>(std::abs(x(i)), std::abs(y5(i)));
					const double r = e(i) / sc;
					errNorm += r * r;
				}
				double err = std::sqrt(errNorm / e.size());

				// Adaptive step size control

				// Accept
				if (err <= 1.0 && std::isfinite(err)) {
					dt_used = dt; // Store the actual step size used for this step

					// Update step size for next iteration
					const double denom = std::max<double>(err, 1e-10); // prevent division by zero
					double fac = safety * std::pow(denom, -0.2);	   // exponent for 5th order method
					fac = std::clamp(fac, fac_min, fac_max);		   // limit step size change
					dt = std::clamp(dt * fac, h_min, h_max);		   // update step size
					return y5;
				}
				// Reject
				else {
					double denom = (std::isfinite(err)
						? std::max<double>(err, 1e-16) : 1e16);	 // prevent division by zero & NaN
					double fac = safety * std::pow(denom, -0.2); // exponent for 4th order method
					fac = std::clamp(fac, fac_min, fac_max);
					dt = std::clamp(dt * fac, h_min, h_max);
				}
			}

			// If it reaches here, it failed to converge after many attempts
			// Just putting this here as a policy choice honestly, return the best effort or throw
			throw std::runtime_error("RK45 failed to converge after maximum attempts");
		}

		// Implicit Euler method
		template<typename Func>
		inline VecX implicit_euler(const VecX& x, double t, double dt, Func&& f, int maxIter = 100, double tol = 1e-6) {
			VecX x_new = x; // Initial guess

			// Numerical Jacobian for Newton-Raphson
			auto J = [&](const VecX& x_guess) -> MatX {
				const double eps_rel = std::sqrt(std::numeric_limits<double>::epsilon());
				VecX f_0 = f(t + dt, x_guess);

				int n = (int)x_guess.size();
				MatX J_full = MatX::Zero(n, n);

				for (int i = 0; i < n; ++i) {
					VecX x_pert = x_guess;
					double h = eps_rel * std::max(1.0, std::abs(x_guess(i)));
					x_pert(i) += h;
					VecX f_i = f(t + dt, x_pert);
					J_full.col(i) = (f_i - f_0) / h; // Finite difference approximationof df/dx column i
				}

				return MatX::Identity(n, n) - dt * J_full; // J = I - dt * df/dx
				};

			// The function g(x_guess) = 0 that we want to solve for the implicit Euler step
			auto g = [&](const VecX& x_guess) { return x_guess - x - dt * f(t + dt, x_guess); };

			// Use Newton-Raphson to solve for x_new such that g(x_new) = 0
			newton_raphson(g, J, x_new, maxIter, tol);

			x_new = x + dt * f(t + dt, x_new); // Final update using the implicit Euler formula
			return x_new;
		}

		// Implicit Midpoint method
		template<typename Func>
		inline VecX implicit_midpoint(const VecX& x, double t, double dt, Func&& f, int maxIter = 100, double tol = 1e-6) {
			VecX x_new = x; // Initial guess

			// Numerical Jacobian for Newton-Raphson
			auto J = [&](const VecX& x_guess) -> MatX {
				const double eps_rel = std::sqrt(std::numeric_limits<double>::epsilon());
				VecX f_0 = f(t + dt / 2.0, (x + x_guess) / 2.0);

				int n = (int)x_guess.size();
				MatX J_full = MatX::Zero(n, n);

				for (int i = 0; i < n; ++i) {
					VecX x_pert = x_guess;
					double h = eps_rel * std::max(1.0, std::abs(x_guess(i)));
					x_pert(i) += h;
					VecX f_i = f(t + dt / 2.0, (x + x_pert) / 2.0);
					J_full.col(i) = (f_i - f_0) / h; // Finite difference approximationof df/dx column i
				}

				return MatX::Identity(n, n) - 0.5 * dt * J_full; // J = I - dt * df/dx
				};

			// The function g(x_guess) = 0 that we want to solve for the implicit midpoint step
			auto g = [&](const VecX& x_guess) { return x_guess - x - dt * f(t + dt / 2.0, (x + x_guess) / 2.0); };

			// Use Newton-Raphson to solve for x_new such that g(x_new) = 0
			newton_raphson(g, J, x_new, maxIter, tol);

			x_new = x + dt * f(t + dt / 2.0, (x + x_new) / 2.0); // Final update using the implicit midpoint formula
			return x_new;
		}

		// Gauss-Legendre Runge-Kutta method (2 stages, 4th order)
		template<typename Func>
		inline VecX GLRK2(const VecX& x, double t, double dt, Func&& f, int maxIter = 100, double tol = 1e-6) {
			size_t n = x.size();

			// Coefficients for the 2-stage Gauss-Legendre method (4th order)
			const VecX c{ 0.5 - std::sqrt(3.0) / 6.0, 0.5 + std::sqrt(3.0) / 6.0 }; // Stage time fractions
			const double b = 0.5; // Weights for final update
			Mat2 A = Mat2::Zero();
			A << 0.25, 0.25 - std::sqrt(3.0) / 6.0,
				0.25 + std::sqrt(3.0) / 6.0, 0.25;

			// Initial guess for the stage values k1, k2, k3
			VecX k = VecX::Zero(n * 2); // 3 stages
			for (int i = 0; i < 2; ++i) {
				VecX x_i = x + dt * (A(i, 0) * k.segment(0, n) + A(i, 1) * k.segment(n, n)); // Initial guess for stage i
				k.segment(i * n, n) = f(t + c(i) * dt, x_i);
			}

			// System of equations to solve for the stage values k1, k2:
			auto g = [&](const VecX& k_guess) {
				VecX x1_guess = x + dt * (A(0, 0) * k_guess.segment(0, n) + A(0, 1) * k_guess.segment(n, n));
				VecX x2_guess = x + dt * (A(1, 0) * k_guess.segment(0, n) + A(1, 1) * k_guess.segment(n, n));
				VecX g1 = k_guess.segment(0, n) - f(t + c(0) * dt, x1_guess);
				VecX g2 = k_guess.segment(n, n) - f(t + c(1) * dt, x2_guess);
				VecX g(k_guess.size());
				g << g1, g2;
				return g;
				};

			auto J = [&](const VecX& k_guess) { return J_GLRK(k_guess, g); };

			newton_raphson(g, J, k, maxIter, tol);

			VecX x_f = x + dt * (b * k.segment(0, n) + b * k.segment(n, n));
			return x_f;
		}

		// Gauss-Legendre Runge-Kutta method (3 stages, 6th order)
		template<typename Func>
		inline VecX GLRK3(const VecX& x, double t, double dt, Func&& f, int maxIter = 100, double tol = 1e-6) {
			size_t n = x.size();

			// Coefficients for the 3-stage Gauss-Legendre method (6th order)
			const VecX c{ 0.5 - std::sqrt(15.0) / 10.0, 0.5, 0.5 + std::sqrt(15.0) / 10.0 }; // Stage time fractions
			const VecX b{ 5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0 }; // Weights for final update
			Mat3 A = Mat3::Zero();
			A << 5.0 / 36.0, 2.0 / 9.0 - std::sqrt(15.0) / 15.0, 1.0 / 36.0 - std::sqrt(15.0) / 30.0,
				5.0 / 36.0 + std::sqrt(15.0) / 24.0, 2.0 / 9.0, 5.0 / 36.0 - std::sqrt(15.0) / 24.0,
				5.0 / 36.0 + std::sqrt(15.0) / 30.0, 2.0 / 9.0 + std::sqrt(15.0) / 15.0, 5.0 / 36.0;

			// Initial guess for the stage values k1, k2, k3
			VecX k = VecX::Zero(n * 3); // 3 stages
			for (int iter = 0; iter < 1; ++iter) { // Picard iteration to refine initial guess
				for (int i = 0; i < 3; ++i) {
					VecX x_i = x + dt * (A(i, 0) * k.segment(0, n) + A(i, 1) * k.segment(n, n) + A(i, 2) * k.segment(2 * n, n)); // Initial guess for stage i
					k.segment(i * n, n) = f(t + c(i) * dt, x_i);
				}
			}

			// System of equations to solve for the stage values k1, k2, k3:
			auto g = [&](const VecX& k_guess) {
				VecX x1_guess = x + dt * (A(0, 0) * k_guess.segment(0, n) + A(0, 1) * k_guess.segment(n, n) + A(0, 2) * k_guess.segment(2 * n, n));
				VecX x2_guess = x + dt * (A(1, 0) * k_guess.segment(0, n) + A(1, 1) * k_guess.segment(n, n) + A(1, 2) * k_guess.segment(2 * n, n));
				VecX x3_guess = x + dt * (A(2, 0) * k_guess.segment(0, n) + A(2, 1) * k_guess.segment(n, n) + A(2, 2) * k_guess.segment(2 * n, n));
				VecX g1 = k_guess.segment(0, n) - f(t + c(0) * dt, x1_guess);
				VecX g2 = k_guess.segment(n, n) - f(t + c(1) * dt, x2_guess);
				VecX g3 = k_guess.segment(2 * n, n) - f(t + c(2) * dt, x3_guess);
				VecX g(k_guess.size());
				g << g1, g2, g3;
				return g;
				};

			// Jacobian of the system g(k) = 0 with respect to k
			auto J = [&](const VecX& k_guess) { return J_GLRK(k_guess, g); };

			// Solve the nonlinear system for the stage values using Newton-Raphson
			newton_raphson(g, J, k, maxIter, tol);

			// Compute the final update for x using the stage values
			VecX x_f = x + dt * (b(0) * k.segment(0, n) + b(1) * k.segment(n, n) + b(2) * k.segment(2 * n, n));
			return x_f;
		}

	private:

		// Generic Newton-Raphson solver for systems of nonlinear equations g(k) = 0
		template<typename G, typename J>
		VecX newton_raphson(G&& g, J&& Jf, VecX x0, int maxIter, double tol) {
			VecX x = x0;
			for (int iter = 0; iter < maxIter; ++iter) {
				VecX gx = g(x);
				if (gx.norm() < tol) {
					return x;
				}
				MatX Jx = Jf(x);
				Eigen::FullPivLU<MatX> lu(Jx);
				if (!lu.isInvertible()) {
					throw std::runtime_error("Jacobian is singular during Newton-Raphson iteration " + std::to_string(iter + 1));
				}
				VecX delta = lu.solve(-gx);

				double lambda = 1.0; // Line search parameter
				double norm_gx = gx.norm(); // Norm of the residual before the update

				VecX x_trial;

				// Backtracking line search to ensure we are making progress (convergence)
				while (lambda > 1e-6) {
					x_trial = x + lambda * delta;
					VecX gx_trial = g(x_trial);

					// Check if the new guess has a smaller residual norm
					if (gx_trial.norm() < norm_gx) {
						x = x_trial; // Accept the update
						break;
					}
					lambda *= 0.5; // Reduce step size
				}
				if (lambda <= 1e-6) {
					throw std::runtime_error("Line search failed during Newton-Raphson iteration " + std::to_string(iter + 1));
				}
			}
			VecX gx = g(x);
			throw std::runtime_error("Newton-Raphson failed to converge after " + std::to_string(maxIter) + " iterations, final residual norm: " + std::to_string(g(x).norm()));
		}

		// Numerical Jacobian for the system g(k) = 0
		template<typename GFunc>
		MatX J_GLRK(const VecX& k_guess, GFunc&& g) {
			// Numerical Jacobian for the system g(k) = 0
			const double eps_rel = std::sqrt(std::numeric_limits<double>::epsilon());
			VecX g_0 = g(k_guess);
			int n = (int)k_guess.size();
			MatX J_full = MatX::Zero(n, n);

			// Compute the Jacobian matrix using finite differences
			for (int i = 0; i < n; ++i) {
				VecX k_pert = k_guess;
				double h = eps_rel * std::max(1.0, std::abs(k_guess(i)));
				k_pert(i) += h;
				VecX g_i = g(k_pert);
				J_full.col(i) = (g_i - g_0) / h; // FDA of dg/dk column i
			}
			return J_full;
		}

	};

	// Partial Differential Equation (PDE) solvers
	class MATHLIB_API PDE {
	public:

		// Explicit finite difference method for 1D heat equation: u_t = alpha * u_xx
		inline VecX fdmStep(const VecX& u, double dx, double dt, double alpha) {
			int n = (int)u.size();
			VecX u_new = u;
			double r = alpha * dt / (dx * dx);
			for (int i = 1; i < n - 1; ++i) {
				u_new(i) = u(i) + r * (u(i + 1) - 2 * u(i) + u(i - 1));
			}
			return u_new;
		}
	};
}