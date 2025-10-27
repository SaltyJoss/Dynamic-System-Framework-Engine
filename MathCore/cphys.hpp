#pragma once
#include "mathlib/Constants/cmath.hpp"

namespace physlib::cphys {

    // Fundamental physics constants
    inline constexpr double G = 6.67430e-11;
    inline constexpr double mu_Earth = 3.986004418e14;
    inline constexpr double g_Earth = 9.80665;
    inline constexpr double g_Moon = 1.6220013;
    inline constexpr double g_Mars = 3.72076;
    inline constexpr double c = 2.99792458e8;

    // Extra physics constants
    inline constexpr double microgravity_threshold = 1e-6;    // 1e-3 to 1e-6, so 1e-6
    inline constexpr double air_density = 1.225;
    inline constexpr double vacuum_density = 0.0;

    // Derived constants from c_math (automatically convert to double)
    inline constexpr double p_pi = mathlib::cmath::pi;
    inline constexpr double p_two_pi = mathlib::cmath::two_pi;
    inline constexpr double p_pi_over_2 = mathlib::cmath::pi_over_2;
    inline constexpr double p_pi_over_4 = mathlib::cmath::pi_over_4;
    inline constexpr double p_one_over_pi = mathlib::cmath::inv_pi;
    inline constexpr double p_two_over_pi = mathlib::cmath::two_over_pi;
}
