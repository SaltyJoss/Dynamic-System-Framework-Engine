#pragma once

#include "MathLibAPI.h"

namespace constants {
	// Mathematical constants
	class MATHLIB_API MathConstants {
	public:
		static constexpr long double e = 2.718281828459045235L;				// Euler's number
		static constexpr long double pi = 3.141592653589793238L;				// Pi
		static constexpr long double phi = 1.618033988749894848L;			// Golden ratio
		static constexpr long double sqrt2 = 1.414213562373095049L;			// Square root of 2
		static constexpr long double inv_sqrt2 = 0.707106781186547524L;		// Inverse square root of 2
		static constexpr long double ln_2 = 0.693147180559945309L;			// Natural logarithm of 2
		static constexpr long double ln_10 = 2.30258509299404568402L;		// Natural logarithm of 10
		static constexpr long double log2_e = 1.442695040888963407L;			// Base-2 logarithm of e
		static constexpr long double log10_e = 0.434294481903251804L;		// Base-10 logarithm of e
		static constexpr long double pi_over_2 = 1.570796326794896619L;		// Pi over 2
		static constexpr long double pi_over_4 = 0.785398163397448309L;		// Pi over 4
		static constexpr long double inv_pi = 0.318309886183790671L;			// 1 over Pi
		static constexpr long double two_over_pi = 0.636619772367581382L;	// 2 over Pi
		static constexpr long double two_pi = 6.283185307179586477L;			// 2 times Pi
	};
}