// PxM/MathLib DualNumbers.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"

namespace mathlib {
	// Template version of dual number (single variable)
	template<typename Scalar>
	class DualNumber_T {
	public:
		DualNumber_T(Scalar real = Scalar(0), Scalar dual = Scalar(0)) : real(real), dual(dual) {}
		Scalar real;
		Scalar dual;
	};

	// Addition
	template<typename Scalar>
	inline DualNumber_T<Scalar> operator+(
		const DualNumber_T<Scalar>& a,
		const DualNumber_T<Scalar>& b
	) {
		return DualNumber_T<Scalar>(a.real + b.real, a.dual + b.dual);
	}

	// Subtraction
	template<typename Scalar>
	inline DualNumber_T<Scalar> operator-(
		const DualNumber_T<Scalar>& a,
		const DualNumber_T<Scalar>& b
	) {
		return DualNumber_T<Scalar>(a.real - b.real, a.dual - b.dual);
	}

	// Multiplication
	template<typename Scalar>
	inline DualNumber_T<Scalar> operator*(
		const DualNumber_T<Scalar>& a,
		const DualNumber_T<Scalar>& b
	) {
		return DualNumber_T<Scalar>(
			a.real * b.real,
			a.real * b.dual + a.dual * b.real
		);
	}

	// Division
	template<typename Scalar>
	inline DualNumber_T<Scalar> operator/(
		const DualNumber_T<Scalar>& a,
		const DualNumber_T<Scalar>& b
	) {
		return DualNumber_T<Scalar>(
			a.real / b.real,
			(a.dual * b.real - a.real * b.dual) / (b.real * b.real)
		);
	}

	// Exponential function
	template<typename Scalar>
	inline DualNumber_T<Scalar> exp(
		const DualNumber_T<Scalar>& a
	) {
		Scalar expReal = std::exp(a.real);
		return DualNumber_T<Scalar>(expReal, expReal * a.dual);
	}

	// Power function (x^n)
	template<typename Scalar>
	inline DualNumber_T<Scalar> pow(<
		const DualNumber_T<Scalar>& a,
		Scalar n
	) {
		Scalar realPow = std::pow(a.real, n);
		return DualNumber_T<Scalar>(
			realPow,
			n * std::pow(a.real, n - Scalar(1)) * a.dual
		);
	}

	// Square root function
	template<typename Scalar>
	inline DualNumber_T<Scalar> sqrt(
		const DualNumber_T<Scalar>& a
	) {
		Scalar sqrtReal = std::sqrt(a.real);
		return DualNumber_T<Scalar>(
			sqrtReal,
			Scalar(0.5) * a.dual / sqrtReal
		);
	}

	// Sine function
	template<typename Scalar>
	inline DualNumber_T<Scalar> sin(
		const DualNumber_T<Scalar>& a
	) {
		return DualNumber_T<Scalar>(
			std::sin(a.real),
			std::cos(a.real) * a.dual
		);
	}

	// Cosine function
	template<typename Scalar>
	inline DualNumber_T<Scalar> cos(
		const DualNumber_T<Scalar>& a
	) {
		return DualNumber_T<Scalar>(
			std::cos(a.real),
			-std::sin(a.real) * a.dual
		);
	}

	// Tangent function
	template<typename Scalar>
	inline DualNumber_T<Scalar> tan(
		const DualNumber_T<Scalar>& a
	) {
		Scalar cosReal = std::cos(a.real);
		return DualNumber_T<Scalar>(
			std::tan(a.real),
			a.dual / (cosReal * cosReal)
		);
	}

	// Arctangent function
	template<typename Scalar>
	inline DualNumber_T<Scalar> atan(
		const DualNumber_T<Scalar>& a
	) {
		return DualNumber_T<Scalar>(
			std::atan(a.real),
			a.dual / (1 + a.real * a.real)
		);
	}

	// Smoothstep function for transition from 0 to 1 as x goes from 0 to 1
	template<typename Scalar>
	inline DualNumber_T<Scalar> smoothStep(
		DualNumber_T<Scalar> x
	) {
		return x * x * (DualNumber_T<Scalar>(3) - DualNumber_T<Scalar>(2) * x);
	}

	// Smoothstep function with edge parameters
	// * Derivative of smoothstep with respect to t is 6*t*(1-t), and dt/da = 1/(edge1 - edge0)
	template<typename Scalar>
	inline DualNumber_T<Scalar> smoothStep(
		DualNumber_T<Scalar> edge0,
		DualNumber_T<Scalar> edge1,
		DualNumber_T<Scalar> a
	) {
		DualNumber_T<Scalar> t = (a - edge0) / (edge1 - edge0);
		return DualNumber_T<Scalar>(
			t.real * t.real * (Scalar(3) - Scalar(2) * t.real),
			t.dual * (Scalar(6) * t.real * (Scalar(1) - t.real))
		);
	}

	// Sigmoid function
	template<typename Scalar>
	inline DualNumber_T<Scalar> sigmoid(
		DualNumber_T<Scalar> a
	) {
		Scalar expNegReal = std::exp(-a.real);
		Scalar sigmoidReal = Scalar(1) / (Scalar(1) + expNegReal);
		return DualNumber_T<Scalar>(
			sigmoidReal,
			a.dual * sigmoidReal * (Scalar(1) - sigmoidReal)
		);
	}
}