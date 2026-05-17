// PxM/MathLib M_DualNumbers.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"

namespace mathlib {
	// Template version of dual number
	template<typename Scalar, size_t NVar>
	class DualNumber_T {
	public:
		DualNumber_T(
			Scalar real = Scalar(0),
			const std::array<Scalar, NVar>& dual = std::array<Scalar, NVar>()
		) : real(real), dual(dual) {
		}

		DualNumber_T(
			Scalar real,
			std::initializer_list<Scalar> duals
		) : real(real) {
			dual.fill(Scalar(0));
			std::copy_n(
				duals.begin(),
				std::min(duals.size(), NVar),
				dual.begin()
			);
		}

		Scalar real;
		std::array<Scalar, NVar> dual;
	};

	// -----
	// Unary Operators
	// -----

	// Negation
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = -a.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = -a.dual[i];
		}
		return out;
	}

	// Conjugate (negate dual part, keep real part)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> conjugate(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = -a.dual[i];
		}
		return out;
	}

	// Identity
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		const DualNumber_T<Scalar, NVar>& a
	) {
		return a;
	}

	// -----
	// Comparison Operators (compare only the real part)
	// -----

	// Equality
	template<typename Scalar, size_t NVar>
	inline bool operator==(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.real == b.real;
	}

	// Inequality
	template<typename Scalar, size_t NVar>
	inline bool operator!=(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return !(a == b);
	}

	// Less than
	template<typename Scalar, size_t NVar>
	inline bool operator<(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.real < b.real;
	}

	// Less than or equal
	template<typename Scalar, size_t NVar>
	inline bool operator<=(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.real <= b.real;
	}

	// Greater than
	template<typename Scalar, size_t NVar>
	inline bool operator>(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		return a.real > b.real;
	}

	// Greater than or equal
	template<typename Scalar, size_t NVar>
	inline bool operator>=(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.real >= b.real;
	}
	
	// -----
	// Scalar Interactions
	// -----

	// Scalar Addition (dual + scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar b
	) {
		DualNumber_T<Scalar, NVar> out = a;
		out.real += b;
		return out;
	}

	// Scalar Addition (scalar + dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		Scalar a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		return b + a; // Reuse dual + scalar
	}

	// Scalar Subtraction (dual - scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar b
	) {
		DualNumber_T<Scalar, NVar> out = a;
		out.real -= b;
		return out;
	}

	// Scalar Subtraction (scalar - dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		Scalar a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a - b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = -b.dual[i];
		}
		return out;
	}

	// Scalar Multiplication (dual * scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar b
		) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real * b;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] * b;
		}
		return out;
	}

	// Scalar Multiplication (scalar * dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(
		Scalar a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return b * a; // Reuse dual * scalar
	}

	// Scalar Division (dual / scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar b
		) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real / b;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] / b;
		}
		return out;
	}

	// Scalar Division (scalar / dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(
		Scalar a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a / b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = -a * b.dual[i] / (b.real * b.real);
		}
		return out;
	}

	// -----
	// Dual Number Interactions
	// -----

	// Addition
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real + b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] + b.dual[i];
		}
		return out;
	}

	// Subtraction
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real - b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] - b.dual[i];
		}
		return out;
	}

	// Multiplication
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real * b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.real * b.dual[i] + a.dual[i] * b.real;
		}
		return out;
	}

	// Division
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real / b.real;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = (a.dual[i] * b.real - a.real * b.dual[i]) / (b.real * b.real);
		}
		return out;
	}

	// Power function (x^n)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> pow(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar n
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::pow(a.real, n);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = n * std::pow(a.real, n - Scalar(1)) * a.dual[i];
		}
		return out;
	}

	// Square root function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sqrt(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		Scalar sqrtReal = std::sqrt(a.real);
		out.real = sqrtReal;
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = Scalar(0.5) * a.dual[i] / sqrtReal;
		}
		return out;
	}

	// Sine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sin(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::sin(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = std::cos(a.real) * a.dual[i];
		}
		return out;
	}

	// Cosine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> cos(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::cos(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = -std::sin(a.real) * a.dual[i];
		}
		return out;
	}

	// Tangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> tan(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::tan(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] / (std::cos(a.real) * std::cos(a.real));
		}
		return out;
	}

	// Arctangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> atan(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::atan(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] / (Scalar(1) + a.real * a.real);
		}
		return out;
	}

	// Hyperbolic tangent function (tanh)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> tanh(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::tanh(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = a.dual[i] * (Scalar(1) - out.real * out.real);
		}
		return out;
	}

	// Exponential function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> exp(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = std::exp(a.real);
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = out.real * a.dual[i];
		}
		return out;
	}

	// Smooth step function for smooth interpolation between 0 and 1
	template<typename Scalar>
	inline Scalar smoothStep(
		const Scalar& x
	) {
		return x * x * (Scalar(3) - Scalar(2) * x);
	}

	// Smooth step function for dual numbers (applies smooth step to the real part, scales dual part by the derivative of the smooth step)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> smoothStep(
		const DualNumber_T<Scalar, NVar>& x
	) {
		DualNumber_T<Scalar, NVar> out;
		out.real = smoothStep(x.real);
		Scalar derivative = Scalar(6) * x.real * (Scalar(1) - x.real); // Derivative of smooth step with respect to x
		for (size_t i = 0; i < NVar; ++i) {
			out.dual[i] = derivative * x.dual[i];
		}
		return out;
	}

	// -----
	// Assignment Operators
	// -----

	// Addition assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator+=(
		DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		a.real += b.real;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] += b.dual[i];
		}
		return a;
	}

	// Subtraction assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator-=(
		DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		a.real -= b.real;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] -= b.dual[i];
		}
		return a;
	}

	// Scalar multiplication assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator*=(
		DualNumber_T<Scalar, NVar>& a,
		Scalar b
	) {
		a.real *= b;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] *= b;
		}
		return a;
	}

	// Element-wise multiplication assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator*=(
		DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		Scalar ogReal = a.real;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] = ogReal * b.dual[i] + a.dual[i] * b.real;
		}
		a.real = ogReal * b.real;
		return a;
	}

	// Scalar division assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator/=(
		DualNumber_T<Scalar, NVar>& a,
		Scalar b
	) {
		a.real /= b;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] /= b;
		}
		return a;
	}

	// Element-wise division assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator/=(
		DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		Scalar ogReal = a.real;
		for (size_t i = 0; i < NVar; ++i) {
			a.dual[i] = (a.dual[i] * b.real - ogReal * b.dual[i]) / (b.real * b.real);
		}
		a.real = ogReal / b.real;
		return a;
	}

	// -----
	// Numeric Limits Specialisation for DualNumber_T
	// -----

	// TODO: Add specialisation of std::numeric_limits for DualNumber_T to define properties like infinity, NaN, epsilon, etc. based on the underlying Scalar type.
}