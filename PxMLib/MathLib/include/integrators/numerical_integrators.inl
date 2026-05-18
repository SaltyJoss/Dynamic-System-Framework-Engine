// PxM/MathLib numerical_integrators.inl
#pragma once

namespace integration {
	// Euler method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::eulerStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		return x + dt * f(t, x);
	}

	// Second-order Runge-Kutta method (Midpoint)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::midpointStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt / Scalar(2), x + k1 / Scalar(2));
		return x + k2;
	}

	// Second-order Runge-Kutta method (Heun)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::heunStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt, x + k1);
		return x + (k1 + k2) / Scalar(2);
	}

	// Second-order Runge-Kutta method (Ralston)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::ralstonStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + (Scalar(2) / Scalar(3)) * dt, x + (Scalar(2) / Scalar(3)) * k1);
		return x + (k1 + Scalar(3) * k2) / Scalar(4);
	}

	// Fourth-order Runge-Kutta method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::rk4Step(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt / Scalar(2), x + k1 / Scalar(2));
		mathlib::VecX_T<Scalar> k3 = dt * f(t + dt / Scalar(2), x + k2 / Scalar(2));
		mathlib::VecX_T<Scalar> k4 = dt * f(t + dt, x + k3);
		return x + (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4) / Scalar(6);
	}

	// RK45 method with adaptive step size (Dormand-Prince)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::rk45Step(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar& dt,
		Scalar& dt_used,
		Func&& f,
		Scalar rtol,
		Scalar atol
	) {
		const Scalar safety = 0.9;	// safety factor to prevent aggressive step size changes
		const Scalar fac_min = 0.2;	// minimum factor for reducing step size
		const Scalar fac_max = 5.0;	// maximum factor for increasing step size
		const Scalar h_min = 1e-10;	// minimum allowed step size
		const Scalar h_max = 1.0;	// maximum allowed step size

		dt = std::clamp(dt, h_min, h_max);

		// Try up to 25 attempts to find an acceptable step size
		for (int attempt = 0; attempt < 25; ++attempt) {
			// Butcher tableau for Dormand-Prince method (7 stages, 5th order, SHOULD probably be precomputed as static constants, in a matrix or equiv)

			// Coefficients for error estimation
			const Scalar c2 = 1.0 / 5.0;
			const Scalar c3 = 3.0 / 10.0;
			const Scalar c4 = 4.0 / 5.0;
			const Scalar c5 = 8.0 / 9.0;
			const Scalar c6 = 1.0;
			const Scalar c7 = 1.0;

			// Dormand-Prince coefficients
			const Scalar a21 = 1.0 / 5.0;
			const Scalar a31 = 3.0 / 40.0;
			const Scalar a32 = 9.0 / 40.0;
			const Scalar a41 = 44.0 / 45.0;
			const Scalar a42 = -56.0 / 15.0;
			const Scalar a43 = 32.0 / 9.0;
			const Scalar a51 = 19372.0 / 6561.0;
			const Scalar a52 = -25360.0 / 2187.0;
			const Scalar a53 = 64448.0 / 6561.0;
			const Scalar a54 = -212.0 / 729.0;
			const Scalar a61 = 9017.0 / 3168.0;
			const Scalar a62 = -355.0 / 33.0;
			const Scalar a63 = 46732.0 / 5247.0;
			const Scalar a64 = 49.0 / 176.0;
			const Scalar a65 = -5103.0 / 18656.0;
			const Scalar a71 = 35.0 / 384.0;
			const Scalar a72 = 0.0;
			const Scalar a73 = 500.0 / 1113.0;
			const Scalar a74 = 125.0 / 192.0;
			const Scalar a75 = -2187.0 / 6784.0;
			const Scalar a76 = 11.0 / 84.0;

			// Weights for 4th and 5th order estimates
			const Scalar b1 = 35.0 / 384.0;
			const Scalar b2 = 0.0;
			const Scalar b3 = 500.0 / 1113.0;
			const Scalar b4 = 125.0 / 192.0;
			const Scalar b5 = -2187.0 / 6784.0;
			const Scalar b6 = 11.0 / 84.0;
			const Scalar b1s = 5179.0 / 57600.0;
			const Scalar b2s = 0.0;
			const Scalar b3s = 7571.0 / 16695.0;
			const Scalar b4s = 393.0 / 640.0;
			const Scalar b5s = -92097.0 / 339200.0;
			const Scalar b6s = 187.0 / 2100.0;
			const Scalar b7s = 1.0 / 40.0;

			// Compute the Runge-Kutta stages (DP -> 7 stages)
			const mathlib::VecX_T<Scalar> k1 = f(t, x);
			const mathlib::VecX_T<Scalar> k2 = f(t + c2 * dt, x + dt * (a21 * k1));
			const mathlib::VecX_T<Scalar> k3 = f(t + c3 * dt, x + dt * (a31 * k1 + a32 * k2));
			const mathlib::VecX_T<Scalar> k4 = f(t + c4 * dt, x + dt * (a41 * k1 + a42 * k2 + a43 * k3));
			const mathlib::VecX_T<Scalar> k5 = f(t + c5 * dt, x + dt * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4));
			const mathlib::VecX_T<Scalar> k6 = f(t + c6 * dt, x + dt * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5));
			const mathlib::VecX_T<Scalar> k7 = f(t + c7 * dt, x + dt * (a71 * k1 + a72 * k2 + a73 * k3 + a74 * k4 + a75 * k5 + a76 * k6));

			// Compute 4th and 5th order estimates
			const mathlib::VecX_T<Scalar> y5 = x + dt * ((b1 * k1) + (b3 * k3) + (b4 * k4) + (b5 * k5) + (b6 * k6));   // 5th order estimate
			const mathlib::VecX_T<Scalar> y4 = x + dt * ((b1s * k1) + (b3s * k3) + (b4s * k4) + (b5s * k5) + (b6s * k6) + (b7s * k7)); // 4th order estimate

			const mathlib::VecX_T<Scalar> e = y5 - y4; // Error estimate

			// Compute the error norm
			Scalar errNorm = 0.0;
			for (int i = 0; i < e.size(); ++i) {
				Scalar sc = atol + rtol * std::max<Scalar>(std::abs(x(i)), std::abs(y5(i)));
				const Scalar r = e(i) / sc;
				errNorm += r * r;
			}
			Scalar err = std::sqrt(errNorm / e.size());

			// Adaptive step size control

			// Accept
			if (err <= 1.0 && std::isfinite(err)) {
				dt_used = dt; // Store the actual step size used for this step

				// Update step size for next iteration
				const Scalar denom = std::max<Scalar>(err, 1e-10); // prevent division by zero
				Scalar fac = safety * std::pow(denom, -0.2);	   // exponent for 5th order method
				fac = std::clamp(fac, fac_min, fac_max);		   // limit step size change
				dt = std::clamp(dt * fac, h_min, h_max);		   // update step size
				return y5;
			}
			// Reject
			else {
				Scalar denom = (std::isfinite(err) 
					? std::max<Scalar>(err, 1e-16) : 1e16);	 // prevent division by zero & NaN
				Scalar fac = safety * std::pow(denom, -0.2); // exponent for 4th order method
				fac = std::clamp(fac, fac_min, fac_max);
				dt = std::clamp(dt * fac, h_min, h_max);
			}
		}
		throw std::runtime_error("RK45 failed to converge after maximum attempts");
	}

	// Implicit Euler method
	template<typename Scalar, typename Func, typename JacFunc>
	mathlib::VecX_T<Scalar> NumericalIntegrator::implicitEuler(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		JacFunc&& jac,
		int maxIter,
		Scalar tol
	) {
		// The function g(x_guess) = 0 that we want to solve for the implicit Euler step
		auto g = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::VecX_T<Scalar>& g_out) { g_out = x_guess - x - dt * f(t + dt, x_guess); };
		// Numerical Jacobian for Newton-Raphson
		auto J = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::MatX_T<Scalar>& J_out) {
			int n = (int)x_guess.size();

			mathlib::MatX_T<Scalar> F;
			bool analytical_success = false;

			if constexpr (!std::is_same_v<std::decay_t<JacFunc>, std::nullptr_t>) {
				if constexpr (std::is_pointer_v<std::decay_t<JacFunc>> || requires { bool(jac); }) {
					if (jac) {
						jac(x_guess, F); // User-provided Jacobian
						analytical_success = true;
					}
				}
				else {
					jac(x_guess, F); // User-provided Jacobian
					analytical_success = true;
				}
			}
			if (!analytical_success) {
				if (USE_AD_JACOBIANS == true) {
					printf("Using AD Jacobian for Implicit Euler\n");
					F = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(dt);
							return f(t_eval, x_pert);
						},
						t + dt,
						x_guess
					);
				}
				else {
					printf("Using finite difference Jacobian for Implicit Euler\n");
					F = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + dt, x_pert);
						},
						t + dt,
						x_guess
					);
				}
			}

			J_out = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * F; // J = I - dt * df/dx
		};

		mathlib::VecX_T<Scalar> x0 = x + dt * f(t + dt, x); // Initial guess for Newton-Raphson
		return newtonRaphson(g, J, x0, maxIter, tol);
	}

	// Implicit Midpoint method
	template<typename Scalar, typename Func, typename JacFunc>
	mathlib::VecX_T<Scalar> NumericalIntegrator::implicitMidpoint(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		JacFunc&& jac,
		int maxIter,
		Scalar tol
	) {
		// The function g(x_guess) = 0 that we want to solve for the implicit midpoint step
		auto g = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::VecX_T<Scalar>& g_out) { g_out = x_guess - x - dt * f((t + dt) / Scalar(2), (x + x_guess) / Scalar(2)); };

		// Numerical Jacobian for Newton-Raphson
		auto J = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::MatX_T<Scalar>& J_out) {
			int n = (int)x_guess.size();
			mathlib::MatX_T<Scalar> F;
			bool analytical_success = false;

			if constexpr (!std::is_same_v<std::decay_t<JacFunc>, std::nullptr_t>) {
				if constexpr (std::is_pointer_v<std::decay_t<JacFunc>> || requires { bool(jac); }) {
					if (jac) {
						jac((x + x_guess) / Scalar(2), F); // User-provided Jacobian
						analytical_success = true;
					}
				}
				else {
					jac((x + x_guess) / Scalar(2), F); // User-provided Jacobian
					analytical_success = true;
				}
			}
			if (!analytical_success) {
				if (USE_AD_JACOBIANS == true) {
					printf("Using AD Jacobian for Implicit Midpoint\n");
					F = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							auto x_dual = x.template cast<Dual>();
							Dual t_mid = Dual(t_pert + dt) / Dual(2);
							return f(t_mid , (x_dual + x_pert) / Dual(2));
						},
						t + dt / Scalar(2),
						x_guess
					);
				}
				else {
					printf("Using finite difference Jacobian for Implicit Midpoint\n");
					F = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f((t + dt) / Scalar(2), (x + x_pert) / Scalar(2));
						},
						t + dt / Scalar(2),
						x_guess
					);
				}
			}
			J_out = mathlib::MatX_T<Scalar>::Identity(n, n) - Scalar(0.5) * dt * F; // J = I - dt * df/dx
		};

		mathlib::VecX_T<Scalar> x0 = x + dt * f((t + dt) / Scalar(2), x); // Initial guess for Newton-Raphson
		return newtonRaphson(g, J, x0, maxIter, tol);
	}

	// Gauss-Legendre Runge-Kutta method (2 stages, 4th order)
	template<typename Scalar, typename Func, typename JacFunc>
	mathlib::VecX_T<Scalar> NumericalIntegrator::GLRK2(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		JacFunc&& jac,
		int maxIter,
		Scalar tol
	) {
		using std::sqrt;

		const Eigen::Index n = x.size();

		// Coefficients for the 2-stage Gauss-Legendre method (4th order)
		mathlib::VecX_T<Scalar> c(2);
		c <<
			Scalar(0.5) - sqrt(Scalar(3)) / Scalar(6),
			Scalar(0.5) + sqrt(Scalar(3)) / Scalar(6); // Stage time fractions

		mathlib::MatX_T<Scalar> A(2, 2);
		A <<
			Scalar(0.25), Scalar(0.25) - sqrt(Scalar(3)) / Scalar(6),
			Scalar(0.25) + sqrt(Scalar(3)) / Scalar(6), Scalar(0.25);

		const Scalar b = Scalar(0.5); // Weights for final update

		// Initial guess for the stage values k1, k2, k3
		mathlib::VecX_T<Scalar> k(2 * n); // 2 stages
		mathlib::VecX_T<Scalar> f0 = f(t, x);
		k.segment(0, n) = f0;
		k.segment(n, n) = f0;

		auto eval_g = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::VecX_T<Scalar>& g) {
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);

			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2);

			mathlib::VecX_T<Scalar> f1 = f(t + c(0) * dt, x1);
			mathlib::VecX_T<Scalar> f2 = f(t + c(1) * dt, x2);

			g.resize(2 * n);
			g.segment(0, n) = k1 - f1;
			g.segment(n, n) = k2 - f2;
		};

		auto eval_j = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::MatX_T<Scalar>& J) {
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2);
			mathlib::MatX_T<Scalar> F1(n, n), F2(n, n);
			bool analytical_success = false;

			// Attempt to use the provided Jacobian function if it's not a nullptr and is callable
			if constexpr (!std::is_same_v<std::decay_t<JacFunc>, std::nullptr_t>) {
				if constexpr (std::is_pointer_v<std::decay_t<JacFunc>> || requires { bool(jac); }) {
					if (jac) {
						jac(x1, F1);
						jac(x2, F2);
						analytical_success = true;
					}
				}
				else {
					// Pure lambda type execution path
					jac(x1, F1);
					jac(x2, F2);
					analytical_success = true;
				}
			}

			if (!analytical_success) {
				if (USE_AD_JACOBIANS == true) {
					printf("Using AD Jacobian for GLRK2\n");
					F1 = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(c(0)) * Dual(dt);
							return f(t_eval, x_pert);
						},
						t + c(0) * dt,
						x1
					);
					F2 = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(c(1)) * Dual(dt);
							return f(t_eval, x_pert);
						},
						t + c(1) * dt,
						x2
					);
				}
				else {
					printf("Using finite difference Jacobian for GLRK2\n");
					F1 = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + c(0) * dt, x_pert);
						},
						t + c(0) * dt,
						x1
					);
					F2 = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + c(1) * dt, x_pert);
						},
						t + c(1) * dt,
						x2
					);
				}
			}

			J.setZero(2 * n, 2 * n);
			J.block(0, 0, n, n) = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * A(0, 0) * F1;
			J.block(0, n, n, n) = -dt * A(0, 1) * F1;
			J.block(n, 0, n, n) = -dt * A(1, 0) * F2;
			J.block(n, n, n, n) = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * A(1, 1) * F2;
		};

		// Solve the nonlinear system for the stage values using Newton-Raphson
		k = newtonRaphson(eval_g, eval_j, k, maxIter, tol);

		// Compute the final update for x using the stage values
		mathlib::VecX_T<Scalar> k1 = k.segment(0, n);
		mathlib::VecX_T<Scalar> k2 = k.segment(n, n);
		mathlib::VecX_T<Scalar> x_f = x + dt * (b * k1 + b * k2);

		// Precautionary check for divergence (NaN or Inf)
		if (!x_f.allFinite()) { throw std::runtime_error("GLRK2 diverged"); }

		return x_f;
	}

	// Gauss-Legendre Runge-Kutta method (3 stages, 6th order)
	template<typename Scalar, typename Func, typename JacFunc>
	mathlib::VecX_T<Scalar> NumericalIntegrator::GLRK3(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		JacFunc&& jac,
		int maxIter,
		Scalar tol
	) {
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;
		using std::sqrt;
		using std::pow;
		using std::abs;
		using std::max;

		if (mathlib::real(tol) < Real(0)) {
			Real dt_r = mathlib::real(dt);
			Real tol_r = max(Real(1e-12), Real(1e-2) * pow(dt_r, Real(7)));
			tol = Scalar(tol_r);
		}
		const Eigen::Index n = x.size();

		// Coefficients for the 3-stage Gauss-Legendre method (6th order)
		mathlib::VecX_T<Scalar> c(3);
		c <<
			Scalar(0.5) - sqrt(Scalar(15)) / Scalar(10),
			Scalar(0.5),
			Scalar(0.5) + sqrt(Scalar(15)) / Scalar(10); // Stage time fractions

		mathlib::Mat3_T<Scalar> A(3, 3);
		A <<
			Scalar(5) / Scalar(36), Scalar(2) / Scalar(9) - sqrt(Scalar(15)) / Scalar(15), Scalar(5) / Scalar(36) - sqrt(Scalar(15)) / Scalar(30),
			Scalar(5) / Scalar(36) + sqrt(Scalar(15)) / Scalar(24), Scalar(2) / Scalar(9), Scalar(5) / Scalar(36) - sqrt(Scalar(15)) / Scalar(24),
			Scalar(5) / Scalar(36) + sqrt(Scalar(15)) / Scalar(30), Scalar(2) / Scalar(9) - sqrt(Scalar(15)) / Scalar(15), Scalar(5) / Scalar(36);

		mathlib::VecX_T<Scalar> b(3);
		b << 5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0; // Weights for final update

		// Initial guess for the stage values k1, k2, k3
		mathlib::VecX_T<Scalar> k(3 * n); // 3 stages
		mathlib::VecX_T<Scalar> f0 = f(t, x);
		k.segment(0, n) = f0;
		k.segment(n, n) = f0;
		k.segment(2 * n, n) = f0;

		auto eval_g = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::VecX_T<Scalar>& g) {
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> k3 = k_guess.segment(2 * n, n);

			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2 + A(0, 2) * k3);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2 + A(1, 2) * k3);
			mathlib::VecX_T<Scalar> x3 = x + dt * (A(2, 0) * k1 + A(2, 1) * k2 + A(2, 2) * k3);

			mathlib::VecX_T<Scalar> f1 = f(t + c(0) * dt, x1);
			mathlib::VecX_T<Scalar> f2 = f(t + c(1) * dt, x2);
			mathlib::VecX_T<Scalar> f3 = f(t + c(2) * dt, x3);

			g.resize(3 * n);

			g.segment(0, n) = k1 - f1;
			g.segment(n, n) = k2 - f2;
			g.segment(2 * n, n) = k3 - f3;
		};

		auto eval_j = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::MatX_T<Scalar>& J) {
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> k3 = k_guess.segment(2 * n, n);

			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2 + A(0, 2) * k3);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2 + A(1, 2) * k3);
			mathlib::VecX_T<Scalar> x3 = x + dt * (A(2, 0) * k1 + A(2, 1) * k2 + A(2, 2) * k3);

			mathlib::MatX_T<Scalar> F1(n, n), F2(n, n), F3(n, n);
			bool analytical_success = false;

			// Attempt to use the provided Jacobian function if it's not a nullptr and is callable
			if constexpr (!std::is_same_v<std::decay_t<JacFunc>, std::nullptr_t>) {
				if constexpr (std::is_pointer_v<std::decay_t<JacFunc>> || requires { bool(jac); }) {
					if (jac) {
						jac(x1, F1);
						jac(x2, F2);
						jac(x3, F3);
						analytical_success = true;
					}
				}
				else {
					// Pure lambda type execution path
					jac(x1, F1);
					jac(x2, F2);
					jac(x3, F3);
					analytical_success = true;
				}
			}

			if (!analytical_success) {
				if (USE_AD_JACOBIANS == true) {
					printf("Using AD Jacobian for GLRK3\n");
					F1 = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(c(0)) * Dual(dt);
							return f(t_eval, x_pert);
						},
						t + c(0) * dt,
						x1
					);
					F2 = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(c(1)) * Dual(dt);
							return f(t_eval, x_pert);
						},
						t + c(1) * dt,
						x2
					);
					F3 = automaticDifferenceJacobian(
						[&](auto t_pert, const auto& x_pert) {
							using Dual = std::decay_t<decltype(x_pert(0))>;
							Dual t_eval = Dual(t_pert) + Dual(c(2)) * Dual(dt);
							return f(t_eval, x_pert);
						},
						t + c(2) * dt,
						x3
					);
				}
				else {
					printf("Using finite difference Jacobian for GLRK3\n");
					F1 = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + c(0) * dt, x_pert);
						},
						t + c(0) * dt,
						x1
					);

					F2 = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + c(1) * dt, x_pert);
						},
						t + c(1) * dt,
						x2
					);

					F3 = finiteDifferenceJacobian(
						[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
							return f(t + c(2) * dt, x_pert);
						},
						t + c(2) * dt,
						x3
					);
				}
			}

			J.setZero(3 * n, 3 * n);
			J.block(0, 0, n, n) = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * A(0, 0) * F1;
			J.block(0, n, n, n) = -dt * A(0, 1) * F1;
			J.block(0, 2 * n, n, n) = -dt * A(0, 2) * F1;
			J.block(n, 0, n, n) = -dt * A(1, 0) * F2;
			J.block(n, n, n, n) = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * A(1, 1) * F2;
			J.block(n, 2 * n, n, n) = -dt * A(1, 2) * F2;
			J.block(2 * n, 0, n, n) = -dt * A(2, 0) * F3;
			J.block(2 * n, n, n, n) = -dt * A(2, 1) * F3;
			J.block(2 * n, 2 * n, n, n) = mathlib::MatX_T<Scalar>::Identity(n, n) - dt * A(2, 2) * F3;
		};

		// Solve the nonlinear system for the stage values using Newton-Raphson
		k = newtonRaphson(eval_g, eval_j, k, maxIter, tol);

		mathlib::VecX_T<Scalar> g_check;
		eval_g(k, g_check);

		auto residual = mathlib::real(g_check.norm());
		//if (residual > 1e-8) { std::cout << "[GLRK3] Large final residual: " << residual << std::endl; }

		// Compute the final update for x using the stage values
		mathlib::VecX_T<Scalar> k1 = k.segment(0, n);
		mathlib::VecX_T<Scalar> k2 = k.segment(n, n);
		mathlib::VecX_T<Scalar> k3 = k.segment(2 * n, n);
		mathlib::VecX_T<Scalar> x_f = x + dt * (b(0) * k1 + b(1) * k2 + b(2) * k3);

		// Precautionary check for divergence (NaN or Inf)
		if (!x_f.allFinite()) { throw std::runtime_error("GLRK3 diverged"); }

		return x_f;
	}

	// Newton-Raphson solver for systems of nonlinear equations g(x) = 0
	template<typename Scalar, typename EvalG, typename EvalJ>
	mathlib::VecX_T<Scalar> NumericalIntegrator::newtonRaphson(
		EvalG&& eval_g,
		EvalJ&& eval_j,
		mathlib::VecX_T<Scalar> x0,
		int maxIter,
		Scalar tol
	) {
		mathlib::VecX_T<Scalar> x = x0, g, x_trial;
		mathlib::VecX_T<Scalar> delta = mathlib::VecX_T<Scalar>::Constant(x0.size(), std::numeric_limits<Scalar>::infinity());
		mathlib::MatX_T<Scalar> J;
		Eigen::ColPivHouseholderQR<mathlib::MatX_T<Scalar>> solver;

		for (int iter = 0; iter < maxIter; ++iter) {
			eval_g(x, g);
			if (!g.allFinite()) {
				throw std::runtime_error(
					"Newton received non-finite residual at iter = "
					+ std::to_string(iter)
					+ ", residual norm = "
					+ std::to_string(g.norm())
				);
			}
			if (g.norm() < tol) { return x; }

			eval_j(x, J);
			if (!J.allFinite()) {
				throw std::runtime_error(
					"Newton received non-finite Jacobian at iter = "
					+ std::to_string(iter)
				);
			}

			solver.compute(J);
			delta = solver.solve(-g);

			if (!delta.allFinite()) {
				throw std::runtime_error(
					"Newton produced non-finite step at iter = "
					+ std::to_string(iter)
				);
			}

			x += delta;
			if (delta.norm() < tol * (Scalar(1) + x.norm())) { return x; }

			if (!x.allFinite()) {
				throw std::runtime_error(
					"Newton state became non-finite at iter = "
					+ std::to_string(iter)
				);
			}
		}

		throw std::runtime_error(
			"Newton-Raphson failed to converge after "
			+ std::to_string(maxIter)
			+ " iterations. Final residual norm = "
			+ std::to_string(g.norm())
			+ ", final step norm = "
			+ std::to_string(delta.norm())
		);
	}

	// Finite difference approximation of the Jacobian matrix df/dx for a vector-valued function f: R^n -> R^m at a point x
	template<typename Scalar, typename Func>
	mathlib::MatX_T<Scalar> NumericalIntegrator::finiteDifferenceJacobian(
		Func&& f,
		Scalar t,
		const mathlib::VecX_T<Scalar>& x
	) {
		const Scalar eps_rel = Scalar(1e-8);
		const int n = static_cast<int>(x.size());
		mathlib::VecX_T<Scalar> f_0 = f(t, x);
		mathlib::MatX_T<Scalar> J(f_0.size(), n);
		// Compute the Jacobian column by column using central differences
		for (int i = 0; i < n; ++i) {
			mathlib::VecX_T<Scalar> x_fwd = x;
			mathlib::VecX_T<Scalar> x_bwd = x;
			Scalar h = eps_rel * std::max(1.0, std::abs(x(i)));
			x_fwd(i) += h;
			x_bwd(i) -= h;
			mathlib::VecX_T<Scalar> f_fwd = f(t, x_fwd);
			mathlib::VecX_T<Scalar> f_bwd = f(t, x_bwd);
			J.col(i) = (f_fwd - f_bwd) / (Scalar(2) * h);
		}
		return J;
	}

	// Automatic Difference Jacobian
	template<typename Scalar, typename Func>
	mathlib::MatX_T<Scalar> NumericalIntegrator::automaticDifferenceJacobian(
		Func&& f,
		Scalar t,
		const mathlib::VecX_T<Scalar>& x
	) {
		const int n = static_cast<int>(x.size());
		mathlib::MatX_T<Scalar> J(n, n);
		for (int i = 0; i < n; ++i) {
			using BaseScalar = typename mathlib::DualTraits<Scalar>::BaseScalar;
			using Dual_T = mathlib::DualNumber_T<BaseScalar, 1>;

			mathlib::VecX_T<Dual_T> x_dual(n);
			for (int k = 0; k < n; ++k) {
				x_dual(k) = Dual_T(
					x(k),
					std::array<Scalar, 1>{
						(k == i) ? Scalar(1) : Scalar(0) 
					}
				);
			}

			Dual_T t_dual(t);
			using ReturnT = decltype(f(t_dual, x_dual));
			ReturnT f_dual = f(t_dual, x_dual);

			for (int j = 0; j < n; ++j) {
				J(j, i) = f_dual(j).dual[0];
			}
		}
		return J;
	}
}