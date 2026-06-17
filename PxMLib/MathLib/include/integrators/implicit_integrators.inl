// PxM/MathLib implicit_integrators.inl
#pragma once

namespace integration {
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
				F = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) {
						return f(t + dt, x_pert);
					},
					t + dt,
					x_guess
				);
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
				F = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f((t + dt) / Scalar(2), (x + x_pert) / Scalar(2)); },
					t + dt / Scalar(2),
					x_guess
				);
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

		// Initial guess for the stage values k1, k2
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
				F1 = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f(t + c(0) * dt, x_pert); },
					t + c(0) * dt,
					x1
				);
				F2 = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f(t + c(1) * dt, x_pert); },
					t + c(1) * dt,
					x2
				);
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
			Real tol_r = max(Real(1e-15), Real(1e-2) * pow(dt_r, Real(7)));
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
			Scalar(5) / Scalar(36) + sqrt(Scalar(15)) / Scalar(30), Scalar(2) / Scalar(9) + sqrt(Scalar(15)) / Scalar(15), Scalar(5) / Scalar(36);

		mathlib::VecX_T<Scalar> b(3);
		b << Scalar(5) / Scalar(18), Scalar(4) / Scalar(9), Scalar(5) / Scalar(18); // Weights for final update

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
				F1 = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f(t + c(0) * dt, x_pert); },
					t + c(0) * dt,
					x1
				);

				F2 = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f(t + c(1) * dt, x_pert); },
					t + c(1) * dt,
					x2
				);

				F3 = finiteDifferenceJacobian(
					[&](Scalar, const mathlib::VecX_T<Scalar>& x_pert) { return f(t + c(2) * dt, x_pert); },
					t + c(2) * dt,
					x3
				);
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
		const Eigen::Index n = x0.size();
		static thread_local mathlib::VecX_T<Scalar> x;
		static thread_local mathlib::VecX_T<Scalar> g;
		static thread_local mathlib::VecX_T<Scalar> delta;
		static thread_local mathlib::MatX_T<Scalar> J;
		if (x.size() != n) {
			x.resize(n);
			g.resize(n);
			delta.resize(n);
			J.resize(n, n);
		}
		x = x0;
		Eigen::PartialPivLU<mathlib::MatX_T<Scalar>> solver;
		for (int iter = 0; iter < maxIter; ++iter) {
			eval_g(x, g);
			if (!g.allFinite()) { throw std::runtime_error("Newton received non-finite residual"); }
			if (g.norm() < tol) { return x; }
			eval_j(x, J);
			if (!J.allFinite()) { throw std::runtime_error("Newton received non-finite Jacobian"); }
			solver.compute(J);
			delta = solver.solve(-g);
			if (!delta.allFinite()) { throw std::runtime_error("Newton produced non-finite step"); }
			x += delta;
			if (delta.norm() < tol * (Scalar(1) + x.norm())) { return x; }
		}
		return x; // Return the last iterate even if we didn't converge, as it may still be a useful approximation
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

		static thread_local mathlib::MatX_T<Scalar> J;
		if (J.rows() != f_0.size() || J.cols() != n) { J.resize(f_0.size(), n); }

		mathlib::VecX_T<Scalar> x_perturbed = x;

		// Compute the Jacobian column by column using forward difference
		for (int i = 0; i < n; ++i) {
			Scalar h = eps_rel * std::max(Scalar(1), Scalar(std::abs(x(i))));
			x_perturbed(i) += h;
			mathlib::VecX_T<Scalar> f_fwd = f(t, x_perturbed);
			x_perturbed(i) = x(i);
			J.col(i) = (f_fwd - f_0) / h;
		}
		return J;
	}
}