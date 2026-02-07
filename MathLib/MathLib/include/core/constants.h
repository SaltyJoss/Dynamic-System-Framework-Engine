#pragma once

#include "MathLibAPI.h"

namespace constants {
	// Mathematical constants
	static constexpr long double e_n			= 2.718281828459045235L; // Euler's number
	static constexpr long double PI				= 3.141592653589793238L; // Pi
	static constexpr long double PHI			= 1.618033988749894848L; // Golden ratio
	static constexpr long double SQRT2			= 1.414213562373095049L; // Square root of 2
	static constexpr long double INV_SQRT2		= 0.707106781186547524L; // Inverse square root of 2
	static constexpr long double ln_2			= 0.693147180559945309L; // Natural logarithm of 2
	static constexpr long double ln_10			= 2.302585092994045684L; // Natural logarithm of 10
	static constexpr long double log2_e			= 1.442695040888963407L; // Base-2 logarithm of e
	static constexpr long double log10_e		= 0.434294481903251804L; // Base-10 logarithm of e
	static constexpr long double PI_OVER_2		= 1.570796326794896619L; // Pi over 2
	static constexpr long double PI_OVER_4		= 0.785398163397448309L; // Pi over 4
	static constexpr long double INV_PI			= 0.318309886183790671L; // 1 over Pi
	static constexpr long double TWO_OVER_PI	= 0.636619772367581382L; // 2 over Pi
	static constexpr long double TWO_PI			= 6.283185307179586477L; // 2 times Pi

	// Derived mathematical constants from const_math (automatically convert to double), cleaner way to access them
	static constexpr double e_n_d			= (double)e_n;
	static constexpr double PI_d			= (double)PI;
	static constexpr double TWO_PI_d		= (double)TWO_PI;
	static constexpr double PI_OVER_2_d		= (double)PI_OVER_2;
	static constexpr double PI_OVER_4_d		= (double)PI_OVER_4;
	static constexpr double ONE_OVER_PI_d	= (double)INV_PI;
	static constexpr double TWO_OVER_PI_d	= (double)TWO_OVER_PI;

	// Fundamental physics constants
	static constexpr double G			= 6.67430e-11;	  // Gravitational constant (m^3 kg^-1 s^-2)
	static constexpr double mu_Earth	= 3.986004418e14; // Standard gravitational parameter for Earth (m^3 s^-2)
	static constexpr double c_0			= 2.99792458e8;	  // Speed of light in vacuum (m/s)

	// zero and micro gravity thresholds (m/s^2)
	static constexpr double g_zero  = 0.0;	// Zero gravity (m/s^2)
	static constexpr double g_micro = 1e-6;	// Microgravity threshold (m/s^2)

	// Standard gravity values for our solar systme (m/s^2)
	static constexpr double g_Sun		= 274.0;	// Standard gravity on Sun's surface (m/s^2)
	static constexpr double g_Mercury	= 3.7;		// Standard gravity on Mercury's surface (m/s^2)
	static constexpr double g_Venus		= 8.87;		// Standard gravity on Venus' surface (m/s^2)
	static constexpr double g_Earth		= 9.80665;	// Standard gravity on Earth's surface (m/s^2)
	static constexpr double g_Mars		= 3.72076;	// Standard gravity on Mars' surface (m/s^2)
	static constexpr double g_Jupiter	= 24.79;	// Standard gravity on Jupiter's surface (m/s^2)
	static constexpr double g_Saturn	= 10.44;	// Standard gravity on Saturn's surface (m/s^2)
	static constexpr double g_Uranus	= 8.69;		// Standard gravity on Uranus' surface (m/s^2)
	static constexpr double g_Neptune	= 11.15;	// Standard gravity on Neptune's surface (m/s^2)
	static constexpr double g_Pluto		= 0.62;		// Standard gravity on Pluto's surface (m/s^2) - "planet"? :D

	// Standard gravity for some of the relevant moons in our solar system (m/s^2)
	static constexpr double g_Moon		= 1.6220013; // Standard gravity on Moon's surface (m/s^2)
	static constexpr double g_Titan		= 1.352;	 // Standard gravity on Titan's surface (m/s^2) - very relevant right now
	static constexpr double g_Enceladus = 0.113;	 // Standard gravity on Enceladus' surface (m/s^2)
	static constexpr double g_Europa	= 1.315;	 // Standard gravity on Europa's surface (m/s^2)
	static constexpr double g_Ganymede	= 1.428;	 // Standard gravity on Ganymede's surface (m/s^2)
	static constexpr double g_Io		= 1.796;	 // Standard gravity on Io's surface (m/s^2)

	// "Standard" gravity values for various space environments (m/s^2)
	static constexpr double g_LowEarthOrbit = 8.0;		// Approximate gravity in Low Earth Orbit (m/s^2)
	static constexpr double g_GeoStationaryOrbit = 3.0;	// Approximate gravity in Geostationary Orbit (m/s^2)
	static constexpr double g_LunarOrbit = 0.5;			// Approximate gravity in Lunar Orbit (m/s^2)
	static constexpr double g_DeepSpace = 1e-9;			// Deep space microgravity (m/s^2)

	// Cool gravity values for blackholes (m/s^2) -> take with a grain of salt as I just want to experiment
	static constexpr double g_SagittariusA = 1e6;			// Approximate gravity near the supermassive black hole at the center of our galaxy (m/s^2)
	static constexpr double g_SolarCorona = 1e-3;			// Approximate gravity in the solar corona (m/s^2)
	static constexpr double g_TravelingAtLightSpeed = 1e-3;	// Hypothetical gravity experienced while traveling at near-light speed (m/s^2)

	// Extra physics constants
	static constexpr double g_threshold = 1e-6;	  // Threshold for microgravity conditions (m/s^2)
	static constexpr double air_density = 1.225;  // Air density at sea level (kg/m^3)
	static constexpr double vacuum_density = 0.0; // Vacuum density (kg/m^3)
}// namespace constants