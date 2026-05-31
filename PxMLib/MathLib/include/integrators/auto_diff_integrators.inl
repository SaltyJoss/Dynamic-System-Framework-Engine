// PxM/MathLib auto_diff_integrators.inl
#pragma once

namespace integration {
	// AD version of Implicit Euler method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::implicitEuler_AD(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		int maxIter,
		typename mathlib::DualTraits<Scalar>::BaseScalar tol
	) {
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;

		// The function g(x_guess) = 0 that we want to solve for the implicit Euler step
		auto g = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::VecX_T<Scalar>& g_out) { g_out = x_guess - x - dt * f(t + dt, x_guess); };
		// Numerical Jacobian for Newton-Raphson
		auto J = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::MatX_T<Real>& J_out) {
			int n = (int)x_guess.size();
			mathlib::MatX_T<Real> F;
			auto perturbation_func = [&](auto t_pert, const auto& x_pert) { 
				using Dual = std::decay_t<decltype(x_pert(0))>;
				Dual t_eval = Dual(t_pert) + Dual(dt);
				return f(t_eval, x_pert); 
			};
			F = automaticDifferenceJacobian(perturbation_func, t + dt, x_guess);
			Real dt_real = static_cast<Real>(dt);
			J_out = mathlib::MatX_T<Real>::Identity(n, n) - dt_real * F; // J = I - dt * df/dx
		};
		mathlib::VecX_T<Scalar> x0 = x + dt * f(t + dt, x); // Initial guess for Newton-Raphson
		return newtonRaphson_AD(g, J, x0, maxIter, tol);
	}

	// AD version of Implicit Midpoint method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::implicitMidpoint_AD(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		int maxIter,
		typename mathlib::DualTraits<Scalar>::BaseScalar tol
	) {
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;

		// The function g(x_guess) = 0 that we want to solve for the implicit midpoint step
		auto g = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::VecX_T<Scalar>& g_out) { g_out = x_guess - x - dt * f((t + dt) / Scalar(2), (x + x_guess) / Scalar(2)); };
		// Numerical Jacobian for Newton-Raphson
		auto J = [&](const mathlib::VecX_T<Scalar>& x_guess, mathlib::MatX_T<Real>& J_out) {
			int n = (int)x_guess.size();
			mathlib::MatX_T<Real> F;
			auto perturbation_func = [&](auto t_pert, const auto& x_pert) {
				using Dual = std::decay_t<decltype(x_pert(0))>;
				mathlib::VecX_T<Dual> x_cast = x.template cast<Dual>();
				mathlib::VecX_T<Dual> x_mid = (x_cast + x_pert) / Dual(2);
				Dual t_mid = t_pert + Dual(static_cast<Real>(dt)) / Dual(2);
				return f(t_mid, x_mid);
			};
			F = automaticDifferenceJacobian(perturbation_func, t, x_guess);
			Real dt_real = static_cast<Real>(dt);
			J_out = mathlib::MatX_T<Real>::Identity(n, n) - dt_real * F; // J = I - dt * df/dx
		};
		mathlib::VecX_T<Scalar> x0 = x + dt * f((t + dt) / Scalar(2), x); // Initial guess for Newton-Raphson
		mathlib::VecX_T<Scalar> res = newtonRaphson_AD(g, J, x0, maxIter, tol);

		// Right-Side Factof Implicit Midpoint (missing from the generic `newtonRaphson_AD` method
		constexpr size_t n = mathlib::DualTraits<Scalar>::Dimension;
		mathlib::MatX_T<Real> Fx;
		auto midpoint_func = [&](auto /*t_pert*/, const auto& x_pert) { return f(t + Scalar(dt) / Scalar(2), x_pert); };
		mathlib::VecX_T<Scalar> x_mid = (x + res) / Scalar(2);
		Fx = automaticDifferenceJacobian(midpoint_func, t, x_mid);
		mathlib::MatX_T<Real> A = mathlib::MatX_T<Real>::Identity(x.size(), x.size()) - Real(0.5) * static_cast<Real>(dt) * Fx;
		mathlib::MatX_T<Real> B = mathlib::MatX_T<Real>::Identity(x.size(), x.size()) + Real(0.5) * static_cast<Real>(dt) * Fx;
		Eigen::FullPivLU<mathlib::MatX_T<Real>> solver(A);
		for (size_t d = 0; d < n; ++d) {
			mathlib::VecX_T<Real> s_old(x.size());
			for (Eigen::Index i = 0; i < x.size(); ++i) { s_old(i) = x(i).dual[d]; }
			mathlib::VecX_T<Real> s_new = solver.solve(B * s_old);
			for (Eigen::Index i = 0; i < x.size(); ++i) { res(i).dual[d] = s_new(i); }
		}
		return res;
	}

	// AD version of Gauss-Legendre Runge-Kutta method (2 stages, 4th order)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::GLRK2_AD(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		int maxIter,
		typename mathlib::DualTraits<Scalar>::BaseScalar tol
	) {
		const Eigen::Index n = x.size();
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;

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

		auto eval_j = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::MatX_T<Real>& J) {
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2);
			mathlib::MatX_T<Real> F1(n, n), F2(n, n);

			F1 = automaticDifferenceJacobian(
				[&](auto t_pert, const auto& x_pert) {
					using Dual = std::decay_t<decltype(x_pert(0))>;
					Dual t_eval = t_pert + Dual(c(0)) * static_cast<Real>(dt); 
					return f(t_eval, x_pert);
				},
				t + c(0) * dt,
				x1
			);
			F2 = automaticDifferenceJacobian(
				[&](auto t_pert, const auto& x_pert) {
					using Dual = std::decay_t<decltype(x_pert(0))>;
					Dual t_eval = t_pert + Dual(c(1)) * static_cast<Real>(dt);
					return f(t_eval, x_pert);
				},
				t + c(1) * dt,
				x2
			);

			Real dt_r = static_cast<Real>(dt);
			mathlib::MatX_T<Real> A_r(2, 2);
			A_r <<
				static_cast<Real>(A(0, 0)), static_cast<Real>(A(0, 1)),
				static_cast<Real>(A(1, 0)), static_cast<Real>(A(1, 1));

			J.setZero(2 * n, 2 * n);
			J.block(0, 0, n, n) = mathlib::MatX_T<Real>::Identity(n, n) - dt_r * A_r(0, 0) * F1;
			J.block(0, n, n, n) = -dt_r * A_r(0, 1) * F1;
			J.block(n, 0, n, n) = -dt_r * A_r(1, 0) * F2;
			J.block(n, n, n, n) = mathlib::MatX_T<Real>::Identity(n, n) - dt_r * A_r(1, 1) * F2;
		};

		// Solve the nonlinear system for the stage values using Newton-Raphson
		k = newtonRaphson_AD(eval_g, eval_j, k, maxIter, tol);

		// Compute the final update for x using the stage values
		mathlib::VecX_T<Scalar> k1 = k.segment(0, n);
		mathlib::VecX_T<Scalar> k2 = k.segment(n, n);
		mathlib::VecX_T<Scalar> x_f = x + dt * (b * k1 + b * k2);

		return x_f;

	}

	// AD version of Gauss-Legendre Runge-Kutta method (3 stages, 6th order)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::GLRK3_AD(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f,
		int maxIter,
		typename mathlib::DualTraits<Scalar>::BaseScalar tol
	) {
		const Eigen::Index n = x.size();
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;

		if (tol < Real(0)) {
			Real dt_r = mathlib::real(dt);
			Real tol_r = max<Real>(Real(1e-12), Real(1e-2) * pow<Real>(dt_r, Real(7)));
			tol = Real(tol_r);
		}

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
			// Extract the stage values k1, k2, k3 from the input guess vector
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> k3 = k_guess.segment(2 * n, n);
			// Compute the stage points x1, x2, x3 based on the current guess for k1, k2, k3
			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2 + A(0, 2) * k3);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2 + A(1, 2) * k3);
			mathlib::VecX_T<Scalar> x3 = x + dt * (A(2, 0) * k1 + A(2, 1) * k2 + A(2, 2) * k3);
			// Compute f at the stage points
			mathlib::VecX_T<Scalar> f1 = f(t + c(0) * dt, x1);
			mathlib::VecX_T<Scalar> f2 = f(t + c(1) * dt, x2);
			mathlib::VecX_T<Scalar> f3 = f(t + c(2) * dt, x3);
			// Compute the residuals for the nonlinear system
			g.resize(3 * n);
			g.segment(0, n) = k1 - f1;
			g.segment(n, n) = k2 - f2;
			g.segment(2 * n, n) = k3 - f3;
		};

		auto eval_j = [&](const mathlib::VecX_T<Scalar>& k_guess, mathlib::MatX_T<Real>& J) {
			// Extract the stage values k1, k2, k3 from the input guess vector
			mathlib::VecX_T<Scalar> k1 = k_guess.segment(0, n);
			mathlib::VecX_T<Scalar> k2 = k_guess.segment(n, n);
			mathlib::VecX_T<Scalar> k3 = k_guess.segment(2 * n, n);
			// Compute the stage points x1, x2, x3 based on the current guess for k1, k2, k3
			mathlib::VecX_T<Scalar> x1 = x + dt * (A(0, 0) * k1 + A(0, 1) * k2 + A(0, 2) * k3);
			mathlib::VecX_T<Scalar> x2 = x + dt * (A(1, 0) * k1 + A(1, 1) * k2 + A(1, 2) * k3);
			mathlib::VecX_T<Scalar> x3 = x + dt * (A(2, 0) * k1 + A(2, 1) * k2 + A(2, 2) * k3);

			mathlib::MatX_T<Real> F1(n, n), F2(n, n), F3(n, n);
			// Compute Jacobians of f at the stage points using automatic differentiation
			F1 = automaticDifferenceJacobian(
				[&](auto t_pert, const auto& x_pert) {
					using Dual = std::decay_t<decltype(x_pert(0))>;
					Dual t_eval = t_pert + Dual(c(0)) * static_cast<Real>(dt);
					return f(t_eval, x_pert);
				},
				t + c(0) * dt,
				x1
			);
			// Compute the Jacobian of f at the second stage using automatic differentiation
			F2 = automaticDifferenceJacobian(
				[&](auto t_pert, const auto& x_pert) {
					using Dual = std::decay_t<decltype(x_pert(0))>;
					Dual t_eval = t_pert + Dual(c(1)) * static_cast<Real>(dt);
					return f(t_eval, x_pert);
				},
				t + c(1) * dt,
				x2
			);
			// Compute the Jacobian of f at the third stage using automatic differentiation
			F3 = automaticDifferenceJacobian(
				[&](auto t_pert, const auto& x_pert) {
					using Dual = std::decay_t<decltype(x_pert(0))>;
					Dual t_eval = t_pert + Dual(c(2)) * static_cast<Real>(dt);
					return f(t_eval, x_pert);
				},
				t + c(2) * dt,
				x3
			);

			Real dt_r = static_cast<Real>(dt);
			mathlib::Mat3_T<Real> A_r(3, 3);
			A_r <<
				static_cast<Real>(A(0, 0)), static_cast<Real>(A(0, 1)), static_cast<Real>(A(0, 2)),
				static_cast<Real>(A(1, 0)), static_cast<Real>(A(1, 1)), static_cast<Real>(A(1, 2)),
				static_cast<Real>(A(2, 0)), static_cast<Real>(A(2, 1)), static_cast<Real>(A(2, 2));

			J.setZero(3 * n, 3 * n);
			J.block(0, 0, n, n) = mathlib::MatX_T<Real>::Identity(n, n) - dt_r * A_r(0, 0) * F1;
			J.block(0, n, n, n) = -dt_r * A_r(0, 1) * F1;
			J.block(0, 2 * n, n, n) = -dt_r * A_r(0, 2) * F1;
			J.block(n, 0, n, n) = -dt_r * A_r(1, 0) * F2;
			J.block(n, n, n, n) = mathlib::MatX_T<Real>::Identity(n, n) - dt_r * A_r(1, 1) * F2;
			J.block(n, 2 * n, n, n) = -dt_r * A_r(1, 2) * F2;
			J.block(2 * n, 0, n, n) = -dt_r * A_r(2, 0) * F3;
			J.block(2 * n, n, n, n) = -dt_r * A_r(2, 1) * F3;
			J.block(2 * n, 2 * n, n, n) = mathlib::MatX_T<Real>::Identity(n, n) - dt_r * A_r(2, 2) * F3;
		};

		// Solve the nonlinear system for the stage values using Newton-Raphson
		k = newtonRaphson_AD(eval_g, eval_j, k, maxIter, tol);
		mathlib::VecX_T<Scalar> g_check;
		eval_g(k, g_check);

		// Compute the final update for x using the stage values
		mathlib::VecX_T<Scalar> k1 = k.segment(0, n);
		mathlib::VecX_T<Scalar> k2 = k.segment(n, n);
		mathlib::VecX_T<Scalar> k3 = k.segment(2 * n, n);
		mathlib::VecX_T<Scalar> x_f = x + dt * (b(0) * k1 + b(1) * k2 + b(2) * k3);

		return x_f;
	}

	// AD version of Newton-Raphson solver for systems of nonlinear equations g(x) = 0
	template<typename Scalar, typename EvalG, typename EvalJ>
	mathlib::VecX_T<Scalar> NumericalIntegrator::newtonRaphson_AD(
		EvalG&& eval_g,
		EvalJ&& eval_j,
		mathlib::VecX_T<Scalar> x0,
		int maxIter,
		typename mathlib::DualTraits<Scalar>::BaseScalar tol
	) {
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;
		const Eigen::Index n = x0.size();
		constexpr size_t NVar = mathlib::DualTraits<Scalar>::Dimension;

		static thread_local mathlib::VecX_T<Real> g_real;
		static thread_local mathlib::VecX_T<Real> delta;
		static thread_local mathlib::MatX_T<Real> J;
		static thread_local mathlib::MatX_T<Real> RHS_seed;
		static thread_local mathlib::MatX_T<Real> corrected_sensitivies;

		if (g_real.size() != n) {
			g_real.resize(n);
			delta.resize(n);
			J.resize(n, n);
			RHS_seed.resize(n, NVar);
			corrected_sensitivies.resize(n, NVar);
		}

		mathlib::VecX_T<Real> x_real = x0.template cast<Real>();
		Eigen::PartialPivLU<mathlib::MatX_T<Real>> solver;
		bool converged = false;

		mathlib::VecX_T<Scalar> g_dual;
		g_dual.resize(n);

		for (int iter = 0; iter < maxIter; ++iter) {
			eval_g(x_real.template cast<Scalar>(), g_dual);
			g_real = g_dual.template cast<Real>();
			if (!g_real.allFinite()) {
				std::ostringstream oss;
				oss << "Newton received non-finite residual at iter = " << iter << '\n';
				std::cerr << oss.str();
				OutputDebugStringA(oss.str().c_str());
				throw std::runtime_error("Newton received non-finite residual at iter = " + std::to_string(iter));
			}
			if (g_real.norm() < tol) { converged = true; break; }

			eval_j(x_real.template cast<Scalar>(), J);
			if (!J.allFinite()) {
				std::ostringstream oss;
				oss << "Newton received non-finite Jacobian at iter = " << iter << '\n';
				std::cerr << oss.str();
				OutputDebugStringA(oss.str().c_str());
				throw std::runtime_error("Newton received non-finite Jacobian at iter = " + std::to_string(iter));
			}

			solver.compute(J);
			delta = solver.solve(-g_real);
			if (!delta.allFinite()) {
				std::ostringstream oss;
				oss << "Newton produced non-finite step at iter = " << iter << '\n';
				std::cerr << oss.str();
				OutputDebugStringA(oss.str().c_str());
				throw std::runtime_error("Newton produced non-finite step at iter = " + std::to_string(iter));
			}

			Real relax = (iter > 10) ? Real(0.5) : Real(1.0); // Simple relaxation strategy after 10 iterations

			x_real += relax * delta;
			Real x_norm = x_real.norm();
			if (delta.norm() < tol * (Real(1) + x_norm)) { converged = true; break; }
		}

		if (!converged) {
			std::ostringstream oss;
			oss << "Newton failed to converge." << '\n'
				<< "iter=" << maxIter << '\n'
				<< "residual=" << g_real.norm() << '\n'
				<< "tol=" << tol << '\n';
			std::cerr << oss.str();
			OutputDebugStringA(oss.str().c_str());
			throw std::runtime_error("Newton-Raphson failed to converge after " + std::to_string(maxIter) + " iterations.");
		}

		mathlib::VecX_T<Scalar> x_final = x_real.template cast<Scalar>();
		eval_j(x_final, J);
		solver.compute(J);
		mathlib::VecX_T<Scalar> g_final;
		eval_g(x_final, g_final);

		for (Eigen::Index j = 0; j < n; ++j) {
			for (size_t d = 0; d < NVar; ++d) { RHS_seed(j, d) = g_final(j).dual[d]; }
		}
		corrected_sensitivies = solver.solve(-RHS_seed);
		for (Eigen::Index i = 0; i < n; ++i) {
			for (size_t d = 0; d < NVar; ++d) { x_final(i).dual[d] = corrected_sensitivies(i, d); }
		}
		return x_final;
	}

	// Automatic Difference Jacobian
	template<typename RealScalar, size_t NVar, typename Func>
	mathlib::MatX_T<RealScalar> NumericalIntegrator::automaticDifferenceJacobian(
		Func&& f,
		mathlib::DualNumber_T<RealScalar, NVar> t,
		const mathlib::VecX_T<mathlib::DualNumber_T<RealScalar, NVar>>& x
	) {
		const int n = static_cast<int>(x.size());
		using Dual_T = mathlib::DualNumber_T<RealScalar, NVar>;
		// Create dual numbers for each input variable in x
		mathlib::VecX_T<Dual_T> x_dual(n);
		for (int k = 0; k < n; ++k) {
			std::array<RealScalar, NVar> seed_array{};
			if (k < static_cast<int>(NVar)) { seed_array[k] = RealScalar(1); } // Set the k-th variable's seed to 1 for forward mode AD, and the rest to 0
			x_dual(k) = Dual_T(mathlib::real(x(k)), seed_array); // Initialise the dual number for x(k) with the real part from x(k) and the appropriate seed for the dual part
		}
		Dual_T t_dual(mathlib::real(t));
		auto f_dual = f(t_dual, x_dual);
		const int m = static_cast<int>(f_dual.size());
		// Construct the Jacobian matrix J where J(j, i) = df_j/dx_i is the dual part of the j-th output corresponding to the seed for the i-th input variable
		mathlib::MatX_T<RealScalar> J(m, n);
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < m; ++j) {
				if (i < static_cast<int>(NVar)) { J(j, i) = f_dual(j).dual[i]; } // The Jacobian entry J(j, i) is the dual part of the j-th output corresponding to the seed for the i-th input variable
				else { J(j, i) = RealScalar(0); } // If i >= NVar, then the seed for that variable is zero, so the Jacobian entry is zero
			}
		}
		return J;
	}
}