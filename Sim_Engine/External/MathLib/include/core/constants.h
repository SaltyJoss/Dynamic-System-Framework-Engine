#pragma once

#include "MathLibAPI.h"

namespace constants {
	// Mathematical constants
	static constexpr long double e = 2.718281828459045235L;				// Euler's number
	static constexpr long double PI = 3.141592653589793238L;			// Pi
	static constexpr long double PHI = 1.618033988749894848L;			// Golden ratio
	static constexpr long double SQRT2 = 1.414213562373095049L;			// Square root of 2
	static constexpr long double INV_SQRT2 = 0.707106781186547524L;		// Inverse square root of 2
	static constexpr long double ln_2 = 0.693147180559945309L;			// Natural logarithm of 2
	static constexpr long double ln_10 = 2.30258509299404568402L;		// Natural logarithm of 10
	static constexpr long double log2_e = 1.442695040888963407L;		// Base-2 logarithm of e
	static constexpr long double log10_e = 0.434294481903251804L;		// Base-10 logarithm of e
	static constexpr long double PI_OVER_2 = 1.570796326794896619L;		// Pi over 2
	static constexpr long double PI_OVER_4 = 0.785398163397448309L;		// Pi over 4
	static constexpr long double INV_PI = 0.318309886183790671L;		// 1 over Pi
	static constexpr long double TWO_OVER_PI = 0.636619772367581382L;	// 2 over Pi
	static constexpr long double TWO_PI = 6.283185307179586477L;		// 2 times Pi

	// Derived mathematical constants from const_math (automatically convert to double), cleaner way to access them
	static constexpr double PI_d = (double)PI;
	static constexpr double TWO_PI_d = (double)TWO_PI;
	static constexpr double PI_OVER_2_d = (double)PI_OVER_2;
	static constexpr double PI_OVER_4_d = (double)PI_OVER_4;
	static constexpr double ONE_OVER_PI_d = (double)INV_PI;
	static constexpr double TWO_OVER_PI_d = (double)TWO_OVER_PI;

	// Fundamental physics constants
	double G = 6.67430e-11;             // Gravitational constant (m^3 kg^-1 s^-2)
	double mu_Earth = 3.986004418e14;   // Standard gravitational parameter for Earth (m^3 s^-2)
	double g_Earth = 9.80665;           // Standard gravity on Earth's surface (m/s^2)
	double g_Moon = 1.6220013;          // Standard gravity on Moon's surface (m/s^2)
	double g_Mars = 3.72076;            // Standard gravity on Mars' surface (m/s^2)
	double c = 2.99792458e8;            // Speed of light in vacuum (m/s)

	// Extra physics constants
	double microgravity_threshold = 1e-6;   // Threshold for microgravity conditions (m/s^2)
	double air_density = 1.225;             // Air density at sea level (kg/m^3)
	double vacuum_density = 0.0;            // Vacuum density (kg/m^3)
}