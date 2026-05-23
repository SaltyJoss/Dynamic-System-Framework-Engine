// PxM/MathLib M_DualNumbers.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types_tpl.h"

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Core>

#include <limits>
#include <array>
#include <algorithm>
#include <ostream>

namespace mathlib {
	// Forward declaration of the DualNumber_T template class
	template<typename Scalar, size_t NVar>
	class DualNumber_T;

	// Helper struct to extract the underlying scalar type from a dual number
	template<typename T>
	struct BaseScalar {
		using type = T;
	};

	// Specialisation for dual numbers to extract the underlying scalar type
	template<typename Scalar, size_t NVar>
	struct BaseScalar<mathlib::DualNumber_T<Scalar, NVar>> {
		using type = Scalar;
	};

	// Template version of dual number
	template<typename Scalar, size_t NVar>
	class DualNumber_T {
	public:
		DualNumber_T(const Scalar& real = Scalar(0))
			: real(real) { dual.fill(Scalar(0)); }
		DualNumber_T(const Scalar& real, const std::array<Scalar, NVar>& dual)
			: real(real), dual(dual) {}
		DualNumber_T(Scalar real, std::initializer_list<Scalar> duals)
			: real(real) {
			dual.fill(Scalar(0));
			std::copy_n(
				duals.begin(),
				std::min(duals.size(), static_cast<size_t>(NVar)),
				dual.begin()
			);
		}
		explicit operator Scalar() const { return real; }
		static constexpr size_t Dimension = NVar;
		Scalar real;
		std::array<Scalar, NVar> dual;
	};

	// -----
	// Unary Operators
	// -----

	// Negation
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = -a.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = -a.dual[i]; }
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

	// Comparison between dual numbers (compare only the real part)
	template<typename Scalar, size_t NVar>
	inline bool operator==(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return a.real == b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator!=(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return !(a == b); }
	template<typename Scalar, size_t NVar>
	inline bool operator<(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return a.real < b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator>(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return a.real > b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator<=(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return a.real <= b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator>=(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) { return a.real >= b.real; }
	// Comparison with scalar (scalar on the right)
	template<typename Scalar, size_t NVar>
	inline bool operator==(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return a.real == b; }
	template<typename Scalar, size_t NVar>
	inline bool operator!=(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return !(a == b); }
	template<typename Scalar, size_t NVar>
	inline bool operator<(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return a.real < b; }
	template<typename Scalar, size_t NVar>
	inline bool operator>(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return a.real > b; }
	template<typename Scalar, size_t NVar>
	inline bool operator<=(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return a.real <= b; }
	template<typename Scalar, size_t NVar>
	inline bool operator>=(const DualNumber_T<Scalar, NVar>& a, Scalar b) { return a.real >= b; }
	// Comparison with scalar (scalar on left)
	template<typename Scalar, size_t NVar>
	inline bool operator==(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return b == a; }
	template<typename Scalar, size_t NVar>
	inline bool operator!=(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return !(b == a); }
	template<typename Scalar, size_t NVar>
	inline bool operator<(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return a < b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator>(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return a > b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator<=(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return a <= b.real; }
	template<typename Scalar, size_t NVar>
	inline bool operator>=(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return a >= b.real; }

	// -----
	// Scalar Interactions
	// -----

	// Scalar Addition (dual + scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(const DualNumber_T<Scalar, NVar>& a, Scalar b) {
		DualNumber_T<Scalar, NVar> out = a;
		out.real += b;
		return out;
	}
	// Scalar Addition (scalar + dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return b + a; }
	// Scalar Subtraction (dual - scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(const DualNumber_T<Scalar, NVar>& a, Scalar b) {
		DualNumber_T<Scalar, NVar> out = a;
		out.real -= b;
		return out;
	}
	// Scalar Subtraction (scalar - dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(Scalar a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a - b.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = -b.dual[i]; }
		return out;
	}
	// Scalar Multiplication (dual * scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(const DualNumber_T<Scalar, NVar>& a, Scalar b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real * b;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] * b; }
		return out;
	}
	// Scalar Multiplication (scalar * dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(Scalar a, const DualNumber_T<Scalar, NVar>& b) { return b * a; }

	// Scalar Division (dual / scalar)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(const DualNumber_T<Scalar, NVar>& a, Scalar b) {
		if (b == Scalar(0)) { throw std::runtime_error("Division by zero in dual number scalar division"); }
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real / b;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] / b; }
		return out;
	}
	// Scalar Division (scalar / dual)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(Scalar a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a / b.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = -a * b.dual[i] / (b.real * b.real); }
		return out;
	}

	// -----
	// Scalar Interactions
	// -----
	
	// Power function (Scalar)
	template<typename Scalar>
	inline Scalar pow(const Scalar& x, const Scalar& n) { return std::pow(x, n); }
	// Sqrt function (Scalar)
	template<typename Scalar>
	inline Scalar sqrt(const Scalar& x) { return std::sqrt(x); }
	// Sine function (Scalar)
	template<typename Scalar>
	inline Scalar sin(const Scalar& x) { return std::sin(x); }
	// Cosine function (Scalar)
	template<typename Scalar>
	inline Scalar cos(const Scalar& x) { return std::cos(x); }
	// Tangent function (Scalar)
	template<typename Scalar>
	inline Scalar tan(const Scalar& x) { return std::tan(x); }
	// Arctangent function (Scalar)
	template<typename Scalar>
	inline Scalar atan(const Scalar& x) { return std::atan(x); }
	// Hyperbolic Tangnet (Scalar)
	template<typename Scalar>
	inline Scalar tanh(const Scalar& x) { return std::tanh(x); }
	// Exponential function (Scalar)
	template<typename Scalar>
	inline Scalar exp(const Scalar& x) { return std::exp(x); }
	// Logarithm function (Scalar)
	template<typename Scalar>
	inline Scalar log(const Scalar& x) { return std::log(x); }

	// -----
	// Dual Number Interactions
	// -----

	// Addition
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator+(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real + b.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] + b.dual[i]; }
		return out;
	}
	// Subtraction
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator-(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real - b.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] - b.dual[i]; }
		return out;
	}
	// Multiplication
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator*(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		out.real = a.real * b.real;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.real * b.dual[i] + a.dual[i] * b.real; }
		return out;
	}
	// Division
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> operator/(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		if (b.real == Scalar(0)) { throw std::runtime_error("Division by zero in dual number division"); }
		DualNumber_T<Scalar, NVar> out;
		const Scalar invDenom = Scalar(1) / b.real;
		const Scalar invDenomSq = invDenom * invDenom;
		out.real = a.real * invDenom;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = (a.dual[i] * b.real - a.real * b.dual[i]) * invDenomSq; }
		return out;
	}
	// Power function (x^n)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> pow(
		const DualNumber_T<Scalar, NVar>& a,
		Scalar n
	) {
		DualNumber_T<Scalar, NVar> out;
		if (a.real == Scalar(0) && n < Scalar(0)) { throw std::runtime_error("Invalid dual power: division by zero"); }
		if (n == Scalar(0)) {
			out.real = Scalar(1);
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = Scalar(0); }
			return out;
		}
		out.real = pow(a.real, n);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = n * pow(a.real, n - Scalar(1)) * a.dual[i]; }
		return out;
	}
	// Square root function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sqrt(const DualNumber_T<Scalar, NVar>& a) {
		if (a.real < Scalar(0)) { throw std::runtime_error("sqrt() domain error for DualNumber_T"); }
		DualNumber_T<Scalar, NVar> out;
		const Scalar sqrtReal = sqrt(a.real);
		out.real = sqrtReal;
		if (sqrtReal == Scalar(0)) {
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = Scalar(0); }
			return out;
		}
		const Scalar inv2sqrt = Scalar(0.5) / sqrtReal;
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] * inv2sqrt; }
		return out;
	}
	// Sine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sin(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = sin(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = cos(a.real) * a.dual[i]; }
		return out;
	}
	// Cosine function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> cos(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = cos(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = -sin(a.real) * a.dual[i]; }
		return out;
	}
	// Tangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> tan(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = tan(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] / (cos(a.real) * cos(a.real)); }
		return out;
	}
	// Arctangent function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> atan(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = atan(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] / (Scalar(1) + a.real * a.real); }
		return out;
	}
	// Hyperbolic tangent function (tanh)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> tanh(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = tanh(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] * (Scalar(1) - out.real * out.real); }
		return out;
	}
	// Exponential function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> exp(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = exp(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = out.real * a.dual[i]; }
		return out;
	}
	// Logarithm function
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> log(const DualNumber_T<Scalar, NVar>& a) {
		if (a.real <= Scalar(0)) { throw std::runtime_error("log() domain error for DualNumber_T"); }
		DualNumber_T<Scalar, NVar> out;
		out.real = log(a.real);
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i] / a.real; }
		return out;
	}

	// -----
	// Smoothing
	// -----

	// Smooth step function for smooth interpolation between 0 and 1
	template<typename Scalar>
	inline Scalar smoothStep(const Scalar& x) { return x * x * (Scalar(3) - Scalar(2) * x); }
	// Smooth step function for dual numbers (applies smooth step to the real part, scales dual part by the derivative of the smooth step)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> smoothStep(const DualNumber_T<Scalar, NVar>& x) {
		DualNumber_T<Scalar, NVar> out;
		out.real = smoothStep(x.real);
		Scalar derivative = Scalar(6) * x.real * (Scalar(1) - x.real); // Derivative of smooth step with respect to x
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = derivative * x.dual[i]; }
		return out;
	}
	// LogSumExp Smooth Max (Scalar)
	template<typename Scalar>
	inline Scalar LSE_smoothMax(const Scalar& a, const Scalar& b, const Scalar& k = Scalar(10)) {
		Scalar m = (a > b) ? a : b;
		return m + log(exp(k * (a - m)) + exp(k * (b - m))) / k;
	}
	// LogSumExp Smooth Max (DualNumber)
	template<typename Scalar, size_t NVar>
	inline Scalar LSE_smoothMax(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b, const Scalar& k = Scalar(15)) {
		Scalar m = (a.real > b.real) ? a.real : b.real;
		return DualNumber_T<Scalar, NVar>(m) + log(exp(k * (a - m)) + exp(k * (b - m))) / k;
	}

	// -----
	// Assignment Operators
	// -----

	// Addition assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator+=(DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		a.real += b.real;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] += b.dual[i]; }
		return a;
	}
	// Addition assignment operator with scalar
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator+=(DualNumber_T<Scalar, NVar>& a, Scalar b) {
		a.real += b;
		return a;
	}
	// Subtraction assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator-=(DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		a.real -= b.real;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] -= b.dual[i]; }
		return a;
	}
	// Subtraction assignment operator with scalar
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator-=(DualNumber_T<Scalar, NVar>& a, Scalar b) {
		a.real -= b;
		return a;
	}
	// Scalar multiplication assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator*=(DualNumber_T<Scalar, NVar>& a, Scalar b) {
		a.real *= b;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] *= b; }
		return a;
	}
	// Element-wise multiplication assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator*=(DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		Scalar ogReal = a.real;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] = ogReal * b.dual[i] + a.dual[i] * b.real; }
		a.real = ogReal * b.real;
		return a;
	}
	// Scalar division assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator/=(DualNumber_T<Scalar, NVar>& a, Scalar b) {
		if (b == Scalar(0)) { throw std::runtime_error("Division by zero in DualNumber_T operator/="); }
		a.real /= b;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] /= b; }
		return a;
	}
	// Element-wise division assignment operator
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar>& operator/=(DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		Scalar ogReal = a.real;
		for (size_t i = 0; i < NVar; ++i) { a.dual[i] = (a.dual[i] * b.real - ogReal * b.dual[i]) / (b.real * b.real); }
		if (b.real == Scalar(0)) { throw std::runtime_error("Division by zero in DualNumber_T operator/="); }
		a.real = ogReal / b.real;
		return a;
	}

	// -----
	// Numeric Limits Specialisation for DualNumber_T
	// -----

	// Absolute value function (Scalar)
	template<typename Scalar>
	inline Scalar abs(const Scalar& a) { return (a < Scalar(0)) ? -a : a; }
	// Maximum value function (Scalar)
	template<typename Scalar>
	inline Scalar max(const Scalar& a, const Scalar& b) { return (a > b) ? a : b; }
	// Minimum value function (Scalar)
	template<typename Scalar>
	inline Scalar min(const Scalar& a, const Scalar& b) { return (a < b) ? a : b; }
	// Sign function (Scalar)
	template<typename Scalar>
	inline Scalar sgn(const Scalar& a) { return (a > Scalar(0)) - (a < Scalar(0)); }
	// Is finite function (Scalar)
	template<typename Scalar>
	inline Scalar isfinite(const Scalar& a) { return std::isfinite(a); }

	// Function to return a DualNumber_T representing infinity (real part is infinity, dual parts are zero)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> numeric_limits_infinity() {
		return DualNumber_T<Scalar, NVar>(std::numeric_limits<Scalar>::infinity(), std::array<Scalar, NVar>());
	}
	// Absolute value function that simply negates the dual number if the real part is negative (this is less smooth but can be more efficient and avoids issues with zero)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> abs(const DualNumber_T<Scalar, NVar>& a) { return abs(a.real); }
	// Absolute value function with epsilon threshold to avoid non-differentiability at zero (scales dual part by the sign of the real part, but treats values within epsilon of zero as zero)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> abs(const DualNumber_T<Scalar, NVar>& a, Scalar epsilon) {
		DualNumber_T<Scalar, NVar> out;
		if (abs(a.real) < epsilon) {
			out.real = Scalar(0);
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = Scalar(0); }
		}
		else {
			out.real = abs(a.real);
			Scalar sign = sgn(a.real);
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = sign * a.dual[i]; }
		}
		return out;
	}
	// Signum function for DualNumber_T (returns the sign of the real part, dual parts are zero)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> sgn(const DualNumber_T<Scalar, NVar>& a) {
		DualNumber_T<Scalar, NVar> out;
		out.real = sgn(a.real); // Signum of the real part
		for (size_t i = 0; i < NVar; ++i) { out.dual[i] = Scalar(0); }
		return out;
	}
	// Minimum function for DualNumber_T (returns the minimum of the real parts, scales dual part by the indicator of which real part is smaller)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> min(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		if (a.real < b.real) {
			out.real = a.real;
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i]; }
		}
		else {
			out.real = b.real;
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = b.dual[i]; }
		}
		return out;
	}
	// Maximum function for DualNumber_T (returns the maximum of the real parts, scales dual part by the indicator of which real part is larger)
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> max(const DualNumber_T<Scalar, NVar>& a, const DualNumber_T<Scalar, NVar>& b) {
		DualNumber_T<Scalar, NVar> out;
		if (a.real > b.real) {
			out.real = a.real;
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = a.dual[i]; }
		}
		else {
			out.real = b.real;
			for (size_t i = 0; i < NVar; ++i) { out.dual[i] = b.dual[i]; }
		}
		return out;
	}
	// Since dual numbers are not complex, the real part is just the real part of the dual number (for non-dual types, this is just the value itself)
	template<typename T>
	inline T real(const T& x) { return x; }
	// Since dual numbers are not complex, the real part is just the real part of the dual number
	template<typename Scalar, size_t NVar>
	inline Scalar real(const DualNumber_T<Scalar, NVar>& a) { return a.real; }
	// Since dual numbers are not complex, the imaginary part is always zero
	template<typename Scalar, size_t NVar>
	inline Scalar imaginary(const DualNumber_T<Scalar, NVar>& /*a*/) { return Scalar(0); }
	// Since dual numbers are not complex, the complex conjugate is just the dual number itself
	template<typename Scalar, size_t NVar>
	inline DualNumber_T<Scalar, NVar> conj(const DualNumber_T<Scalar, NVar>& a) { return a; }
	// Since dual numbers are not complex, the norm is just the absolute value of the real part
	template<typename Scalar, size_t NVar>
	inline Scalar norm(const DualNumber_T<Scalar, NVar>& a) { return abs(a.real); }
	// Since dual numbers are not complex, the squared norm is just the square of the absolute value of the real part
	template<typename Scalar, size_t NVar>
	inline Scalar abs2(const DualNumber_T<Scalar, NVar>& a) { return a.real * a.real; }
	template<typename Scalar, size_t NVar>
	inline bool is_dual_number_v(const DualNumber_T<Scalar, NVar>& /*a*/) { return true; }

	// Traits class to identify dual numbers and extract the base scalar type
	template<typename T>
	struct DualTraits {
		using BaseScalar = T;
		static constexpr bool is_dual = false;
		static constexpr size_t Dimension = 1;
	};
	// Specialization of DualTraits for DualNumber_T
	template<typename Scalar, size_t NVar>
	struct DualTraits<DualNumber_T<Scalar, NVar>> {
		using BaseScalar = Scalar;
		static constexpr bool is_dual = true;
		static constexpr size_t Dimension = NVar;
	};
	// Dual Part
	template<typename Scalar, size_t N>
	Scalar dualPart(const DualNumber_T<Scalar, N>& x) { return x.dual[0]; }
	// Multiple Dual Parts (returns the entire dual part as an array)
	template<typename T, typename Scalar, size_t NVar, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
	inline DualNumber_T<Scalar, NVar> operator*(T a, const DualNumber_T<Scalar, NVar>& b) { return b * static_cast<Scalar>(a); }
	// Multiple Dual Parts (returns the entire dual part as an array)
	template<typename Scalar, size_t NVar>
	inline std::ostream& operator<<(std::ostream& os, const DualNumber_T<Scalar, NVar>& d) {
		os << "{ real: " << d.real << ", dual: [";
		for (size_t i = 0; i < NVar; ++i) {
			os << d.dual[i];
			if (i + 1 < NVar) {
				os << ", ";
			}
		}
		os << "] }";
		return os;
	}
	// Check if the real part and all dual parts are finite
	template<typename Scalar, size_t NVar>
	inline bool isfinite(const DualNumber_T<Scalar, NVar>& x) {
		if (!isfinite(x.real)) { return false; }

		for (size_t i = 0; i < NVar; ++i) {
			if (!isfinite(x.dual[i])) { return false; }
		}

		return true;
	}

	// -----
	// Eigen Matrix Interactions
	// -----

	// Helper function to create a mutable copy of an Eigen expression
	template<typename Derived>
	auto makeMutableCopy(const Eigen::MatrixBase<Derived>& x) -> typename std::decay_t<Derived>::PlainObject { return x.derived().eval(); }
	// Element-wise addition for Eigen matrices of dual numbers
	template<typename Scalar, size_t NVar>
	inline Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic> operator*(
		const Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic>& a,
		const Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic>& b
		) {
		// (a.real() * b.real(), a.real() * b.dual() + a.dual() * b.real())
		return a.derived() * b.derived();
	}
	// Element-wise division for Eigen matrices of dual numbers
	template<typename Scalar, size_t NVar>
	inline Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic> operator/(
		const Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic>& a,
		const Eigen::Matrix<DualNumber_T<Scalar, NVar>, Eigen::Dynamic, Eigen::Dynamic>& b
		) {
		// (a.real() / b.real(), (a.dual() * b.real() - a.real() * b.dual()) / (b.real() * b.real()))
		return a.derived() / b.derived();
	}
}

// Specialisation of Eigen's NumTraits for DualNumber_T to allow Eigen to work with dual numbers in its expressions and algorithms
namespace Eigen {
	template<typename Scalar, size_t NVar>
	struct NumTraits<mathlib::DualNumber_T<Scalar, NVar>> : GenericNumTraits<mathlib::DualNumber_T<Scalar, NVar>> {
		using Dual = mathlib::DualNumber_T<Scalar, NVar>;
		typedef Scalar Real;
		typedef Dual NonInteger;
		typedef Dual Nested;
		typedef Scalar Literal;
		// Define properties of DualNumber_T for Eigen's internal use
		enum {
			IsComplex = 0,
			IsInteger = 0,
			IsSigned = 1,
			RequireInitialization = 1,
			ReadCost = 1,
			AddCost = 3,
			MulCost = 3
		};
		// Set of methods to provide numeric limits for DualNumber_T
		static inline Real epsilon() { return std::numeric_limits<Real>::epsilon(); }
		static inline Real dummy_precision() { return Real(1e-12); }
		static inline Real highest() { return std::numeric_limits<Real>::max(); }
		static inline Real lowest() { return std::numeric_limits<Real>::lowest(); }
	};

	// Specialisation of Eigen's internal traits to ensure that DualNumber_T is treated as a non-arithmetic type
	namespace internal {
		template<typename Scalar, size_t NVar>
		struct is_arithmetic<mathlib::DualNumber_T<Scalar, NVar>> : std::false_type {};

		template<typename Scalar, size_t NVar>
		struct scalar_abs_op<mathlib::DualNumber_T<Scalar, NVar>> {
			using Dual = mathlib::DualNumber_T<Scalar, NVar>;
			typedef Scalar result_type;
			EIGEN_DEVICE_FUNC inline Scalar operator()(const Dual& x) const { return  mathlib::abs(x.real); }
		};

		// Specialisation of scalar_score_coeff_op for DualNumber_T to allow Eigen's internal sorting and selection algorithms to work correctly with dual numbers
		template<typename Scalar, size_t NVar>
		struct scalar_score_coeff_op<mathlib::DualNumber_T<Scalar, NVar>> {
			struct result_type {
				Scalar score;
				result_type(int i = 0) : score(i) {} // Default constructor for compatibility with Eigen's internal mechanisms
				result_type(const mathlib::DualNumber_T<Scalar, NVar>& x) : score(mathlib::abs(x.real)) {}

				friend bool operator <(const result_type& a, const result_type& b) { return a.score < b.score; }
				friend bool operator >(const result_type& a, const result_type& b) { return a.score > b.score; }
				friend bool operator ==(const result_type& a, const result_type& b) { return a.score == b.score; }
			};
			typedef result_type result_type;
			EIGEN_DEVICE_FUNC inline result_type operator()(const mathlib::DualNumber_T<Scalar, NVar>& x) const { return result_type(x); }
		};

		// Specialisation of Eigen's real_impl
		template<typename Scalar, size_t NVar>
		struct real_impl<mathlib::DualNumber_T<Scalar, NVar>> {
			typedef Scalar return_type;
			EIGEN_DEVICE_FUNC static return_type run(const mathlib::DualNumber_T<Scalar, NVar>& x) { return x.real; }
		};
		// Specialisation of Eigen's imag_impl
		template<typename Scalar, size_t NVar>
		struct imag_impl<mathlib::DualNumber_T<Scalar, NVar>> {
			typedef Scalar return_type;
			EIGEN_DEVICE_FUNC static return_type run(const mathlib::DualNumber_T<Scalar, NVar>&) { return Scalar(0); }
		};
		// Specialisation of Eigen's abs2_impl
		template<typename Scalar, size_t NVar>
		struct abs2_impl<mathlib::DualNumber_T<Scalar, NVar>> {
			typedef Scalar return_type;
			EIGEN_DEVICE_FUNC static return_type run(const mathlib::DualNumber_T<Scalar, NVar>& x) { return x.real * x.real; }
		};

		// Specialisation of Eigen's scalar_product_op for DualNumber_T (LHS is dual, RHS is scalar)
		template<typename Scalar, size_t NVar>
		struct scalar_product_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar> {
			typedef mathlib::DualNumber_T<Scalar, NVar> result_type;
			EIGEN_DEVICE_FUNC inline result_type operator()(const mathlib::DualNumber_T<Scalar, NVar>& a, Scalar b) const {
				return a * b; // Use the previously defined operator* for DualNumber_T and scalar
			}
		};
		// Same as above but with the order of the operands reversed (LHS is scalar, RHS is dual)
		template<typename Scalar, size_t NVar>
		struct scalar_product_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>> {
			typedef mathlib::DualNumber_T<Scalar, NVar> result_type;
			EIGEN_DEVICE_FUNC inline result_type operator()(Scalar a, const mathlib::DualNumber_T<Scalar, NVar>& b) const {
				return b * a; // Use the previously defined operator* for DualNumber_T and scalar
			}
		};
	} // namespace internal

	// Standard binary operation traits for DualNumber_T to ensure that operations between dual numbers and scalars yield the correct result type
	// Specialisation for dual + scalar (the result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<mathlib::DualNumber_T<Scalar, NVar>, Scalar, internal::scalar_sum_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};
	// Specialisation for scalar + dual (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<Scalar, mathlib::DualNumber_T<Scalar, NVar>, internal::scalar_sum_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};
	// Specialisation for dual - scalar (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<mathlib::DualNumber_T<Scalar, NVar>, Scalar, internal::scalar_difference_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};
	// Specialisation for scalar - dual (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<Scalar, mathlib::DualNumber_T<Scalar, NVar>, internal::scalar_difference_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};
	// Specialisation for dual * scalar (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<mathlib::DualNumber_T<Scalar, NVar>, Scalar, internal::scalar_product_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};
	// Specialisation for scalar * dual (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<Scalar, mathlib::DualNumber_T<Scalar, NVar>, internal::scalar_product_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>>> {
		typedef mathlib::DualNumber_T<Scalar, NVar> ReturnType;
	};

	// Assignment Operator Traits for DualNumber_T to ensure that compound assignment operations between dual numbers and scalars yield the correct result type
	// Specialisation for dual += scalar (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<mathlib::DualNumber_T<Scalar, NVar>, Scalar, internal::add_assign_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar>> {
		typedef mathlib::DualNumber_T<Scalar, NVar>& ReturnType;
	};
	// Specialisation for Scalar += dual (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<Scalar, mathlib::DualNumber_T<Scalar, NVar>, internal::add_assign_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>>> {
		typedef mathlib::DualNumber_T<Scalar, NVar>& ReturnType;
	};
	// Specialisation for dual -= scalar (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<mathlib::DualNumber_T<Scalar, NVar>, Scalar, internal::sub_assign_op<mathlib::DualNumber_T<Scalar, NVar>, Scalar>> {
		typedef mathlib::DualNumber_T<Scalar, NVar>& ReturnType;
	};
	// Specialisation for Scalar -= dual (result is a dual number)
	template<typename Scalar, size_t NVar>
	struct ScalarBinaryOpTraits<Scalar, mathlib::DualNumber_T<Scalar, NVar>, internal::sub_assign_op<Scalar, mathlib::DualNumber_T<Scalar, NVar>>> {
		typedef mathlib::DualNumber_T<Scalar, NVar>& ReturnType;
	};

	// Specialisation of Eigen's numext functions for DualNumber_T to allow Eigen's algorithms that rely on these functions (like abs, real, imag, conj) to work correctly with dual numbers
	namespace numext {
		template<typename Scalar, size_t NVar>
		inline Scalar real(const mathlib::DualNumber_T<Scalar, NVar>& x) { return x.real; }
		template<typename Scalar, size_t NVar>
		inline Scalar imag(const mathlib::DualNumber_T<Scalar, NVar>&) { return Scalar(0); }
		template<typename Scalar, size_t NVar>
		inline Scalar abs(const mathlib::DualNumber_T<Scalar, NVar>& x) { return mathlib::abs(x.real); }
		template<typename Scalar, size_t NVar>
		inline Scalar abs2(const mathlib::DualNumber_T<Scalar, NVar>& x) { return x.real * x.real; }
		template<typename Scalar, size_t NVar>
		inline Scalar norm1(const mathlib::DualNumber_T<Scalar, NVar>& x) { return  mathlib::abs(x.real); }
		template<typename Scalar, size_t NVar>
		inline mathlib::DualNumber_T<Scalar, NVar> conj(const mathlib::DualNumber_T<Scalar, NVar>& x) { return x; }
	}// namespace numext
} // namespace Eigen