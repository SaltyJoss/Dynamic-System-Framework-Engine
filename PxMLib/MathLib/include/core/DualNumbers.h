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
			const std::array<Scalar, NVar>& duals = std::array<Scalar, NVar>()
		) : m_real(real), m_duals(duals) {
		}

		DualNumber_T(
			Scalar real,
			std::initializer_list<Scalar> duals
		) : m_real(real) {
			std::copy(duals.begin(), duals.end(), m_duals.begin());
		}

		Scalar m_real;
		std::array<Scalar, NVar> m_duals;
	};

	// ----
	// Unary Operators
	// ----

	// Negation
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = -a.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = -a.m_duals[i];
		}
		return out;
	}

	// Conjugate (negate dual part, keep real part)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> conjugate(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = a.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = -a.m_duals[i];
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

	// ----
	// Comparison Operators (compare only the real part)
	// ----

	// Equality
	template<typename Scalar, size_t NVar>
	inline bool operator==(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.m_real == b.m_real;
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
		return a.m_real < b.m_real;
	}

	// Less than or equal
	template<typename Scalar, size_t NVar>
	inline bool operator<=(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.m_real <= b.m_real;
	}

	// Greater than
	template<typename Scalar, size_t NVar>
	inline bool operator>(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		return a.m_real > b.m_real;
	}

	// Greater than or equal
	template<typename Scalar, size_t NVar>
	inline bool operator>=(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
		) {
		return a.m_real >= b.m_real;
	}
	
	// ----
	// Scalar Interactions
	// ----

	// Scalar Addition (dual + scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar b
	) {
		DualNumber_T<Scalar, NVar> out = a;
		out.m_real += b;
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
		out.m_real -= b;
		return out;
	}

	// Scalar Subtraction (scalar - dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(
		Scalar a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = a - b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = -b.m_duals[i];
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
		out.m_real = a.m_real * b;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] * b;
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
		out.m_real = a.m_real / b;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] / b;
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
		out.m_real = a / b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = -a * b.m_duals[i] / (b.m_real * b.m_real);
		}
		return out;
	}

	// ----
	// Dual Number Interactions
	// ----

	// Addition
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(
		const DualNumber_T<Scalar, NVar>& a,
		const DualNumber_T<Scalar, NVar>& b
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = a.m_real + b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] + b.m_duals[i];
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
		out.m_real = a.m_real - b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] - b.m_duals[i];
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
		out.m_real = a.m_real * b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_real * b.m_duals[i] + a.m_duals[i] * b.m_real;
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
		out.m_real = a.m_real / b.m_real;
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = (a.m_duals[i] * b.m_real - a.m_real * b.m_duals[i]) / (b.m_real * b.m_real);
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
		out.m_real = std::pow(a.m_real, n);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = n * std::pow(a.m_real, n - Scalar(1)) * a.m_duals[i];
		}
		return out;
	}

	// Square root function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sqrt(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		Scalar sqrtReal = std::sqrt(a.m_real);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = Scalar(0.5) * a.m_duals[i] / sqrtReal;
		}
		return out;
	}

	// Sine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sin(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = std::sin(a.m_real);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = std::cos(a.m_real) * a.m_duals[i];
		}
		return out;
	}

	// Cosine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> cos(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = std::cos(a.m_real);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = -std::sin(a.m_real) * a.m_duals[i];
		}
		return out;
	}

	// Tangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> tan(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = std::tan(a.m_real);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] / (std::cos(a.m_real) * std::cos(a.m_real));
		}
		return out;
	}

	// Arctangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> atan(
		const DualNumber_T<Scalar, NVar>& a
	) {
		DualNumber_T<Scalar, NVar> out;
		out.m_real = std::atan(a.m_real);
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = a.m_duals[i] / (Scalar(1) + a.m_real * a.m_real);
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
		out.m_real = smoothStep(x.m_real);
		Scalar derivative = Scalar(6) * x.m_real * (Scalar(1) - x.m_real); // Derivative of smooth step with respect to x
		for (size_t i = 0; i < NVar; ++i) {
			out.m_duals[i] = derivative * x.m_duals[i];
		}
		return out;
	}

	// ----
	// Numeric Limits Specialisation for DualNumber_T
	// ----

	// TODO: Add specialisation of std::numeric_limits for DualNumber_T to define properties like infinity, NaN, epsilon, etc. based on the underlying Scalar type.
}