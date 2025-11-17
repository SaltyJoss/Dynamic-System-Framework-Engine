#pragma once
#include "const_math.h"

namespace constants {
    class PhysConstants {
	public:
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

		// Derived constants from const_math (automatically convert to double), cleaner way to access them
        double p_pi = math.pi;
        double p_two_pi = math.two_pi;
        double p_pi_over_2 = math.pi_over_2;
        double p_pi_over_4 = math.pi_over_4;
        double p_one_over_pi = math.inv_pi;
        double p_two_over_pi = math.two_over_pi;

    private:
		MathConstants math;
    };
}
