# ========================================================
# Integrator Comparison Test — KUKA iiwa14 (7-DOF)
# ========================================================
# Duration:  ~105s total
# Purpose:   Exercises all 7 joints through varied trajectory
#            profiles to expose differences between numerical
#            integrators (euler, midpoint, heun, rk4, rk45).
#
# Phases:
#   1 (0–12s)    TRAP  — large point-to-point with high accel
#   2 (12–17s)   HOLD  — settle after TRAP; exposes drift
#   3 (17–47s)   SINE  — single-frequency tracking per joint
#   4 (47–82s)   MSINE — multi-frequency non-periodic excitation
#   5 (82–87s)   HOLD  — settle after MSINE; check energy decay
#   6 (87–102s)  TRAP  — fast opposing reversals (×2)
#
# Joint limits (iiwa14):
#   j1,3,5,7  ±170°      j2,4,6  ±120°
# Load pose:
#   (0, 0.4, 0, -68.5, 0, 68.5, 0)°
# ========================================================

load(robot, iiwa14)
set(integrator, rk4)
wait(1.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# --------------------------------------------------------
# PHASE 1: Trapezoid point-to-point (0 – 12s)
# Asymmetric velocities: base joints slower/heavier,
# wrist joints faster/lighter. This creates coupled
# inertial loading that stresses the integrator.
# --------------------------------------------------------

parallel(12.0)
{
    trajSet(link01, TRAP, 80.0, 45.0, 90.0)
    trajSet(link02, TRAP, -70.0, 40.0, 80.0)
    trajSet(link03, TRAP, 65.0, 50.0, 100.0)
    trajSet(link04, TRAP, -85.0, 45.0, 90.0)
    trajSet(link05, TRAP, 55.0, 65.0, 130.0)
    trajSet(link06, TRAP, -45.0, 55.0, 110.0)
    trajSet(link07, TRAP, 40.0, 75.0, 150.0)
}

wait(12.0)

# --------------------------------------------------------
# PHASE 2: Post-TRAP settle (12 – 17s)
# Hold position after aggressive motion. Poor integrators
# show residual oscillation or energy drift here.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 3: Sinusoidal tracking (17 – 47s)
# Non-commensurate frequencies (no simple ratio between
# any pair) so the combined motion never repeats exactly.
# This prevents integrators from "getting lucky" with
# periodic error cancellation.
# --------------------------------------------------------

parallel(30.0)
{
    trajSet(link01, SINE, 30.0, 40.0, 35.0, 0.13)
    trajSet(link02, SINE, 30.0, -35.0, 30.0, 0.19)
    trajSet(link03, SINE, 30.0, 32.0, 28.0, 0.27)
    trajSet(link04, SINE, 30.0, -42.0, 38.0, 0.31)
    trajSet(link05, SINE, 30.0, 27.0, 22.0, 0.43)
    trajSet(link06, SINE, 30.0, -23.0, 18.0, 0.53)
    trajSet(link07, SINE, 30.0, 20.0, 28.0, 0.67)
}

wait(30.0)

# --------------------------------------------------------
# PHASE 4: Multisine excitation (47 – 82s)
# 2-3 sine components per joint with irrational frequency
# ratios. Creates broadband excitation that tests energy
# conservation, numerical damping, and coupling effects
# over 35 seconds of sustained complex motion.
# --------------------------------------------------------

parallel(35.0)
{
    trajSet(link01, MSINE, 35.0, 15.0,   22.0, 0.11, 0.0,    10.0, 0.31, 90.0)
    trajSet(link02, MSINE, 35.0, -25.0,  18.0, 0.14, 0.0,    9.0, 0.39, 45.0)
    trajSet(link03, MSINE, 35.0, 12.0,   20.0, 0.17, 0.0,    7.0, 0.47, 120.0,   4.0, 0.83, 30.0)
    trajSet(link04, MSINE, 35.0, -18.0,  22.0, 0.19, 0.0,    12.0, 0.43, 60.0,   6.0, 0.79, 150.0)
    trajSet(link05, MSINE, 35.0, 10.0,   16.0, 0.23, 0.0,    10.0, 0.57, 90.0)
    trajSet(link06, MSINE, 35.0, -12.0,  13.0, 0.29, 0.0,    8.0, 0.67, 45.0,    4.0, 1.03, 0.0)
    trajSet(link07, MSINE, 35.0, 8.0,    18.0, 0.33, 0.0,    9.0, 0.73, 60.0)
}

wait(35.0)

# --------------------------------------------------------
# PHASE 5: Post-multisine settle (82 – 87s)
# Hold after the most aggressive phase. Compare residual
# energy between integrators — it should decay to zero.
# Symplectic integrators will show different decay profile.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 6: Fast opposing trapezoids (87 – 102s)
# Three rapid back-and-forth sweeps with increasing
# aggressiveness. Velocity discontinuities at reversal
# points are where integrators diverge most visibly.
# --------------------------------------------------------

# 6a: forward sweep
parallel(5.0)
{
    trajSet(link01, TRAP, -50.0, 85.0, 180.0)
    trajSet(link02, TRAP, 40.0, 75.0, 160.0)
    trajSet(link03, TRAP, -45.0, 80.0, 170.0)
    trajSet(link04, TRAP, 50.0, 85.0, 180.0)
    trajSet(link05, TRAP, -35.0, 95.0, 200.0)
    trajSet(link06, TRAP, 30.0, 75.0, 160.0)
    trajSet(link07, TRAP, -25.0, 105.0, 220.0)
}

wait(5.0)

# 6b: reverse sweep (back through origin)
parallel(5.0)
{
    trajSet(link01, TRAP, 55.0, 90.0, 200.0)
    trajSet(link02, TRAP, -45.0, 80.0, 180.0)
    trajSet(link03, TRAP, 50.0, 85.0, 190.0)
    trajSet(link04, TRAP, -55.0, 90.0, 200.0)
    trajSet(link05, TRAP, 40.0, 100.0, 230.0)
    trajSet(link06, TRAP, -35.0, 80.0, 180.0)
    trajSet(link07, TRAP, 30.0, 110.0, 240.0)
}

wait(5.0)

# 6c: final aggressive snap (highest accel)
parallel(5.0)
{
    trajSet(link01, TRAP, 0.0, 100.0, 250.0)
    trajSet(link02, TRAP, 0.0, 90.0, 220.0)
    trajSet(link03, TRAP, 0.0, 95.0, 240.0)
    trajSet(link04, TRAP, -68.5, 100.0, 250.0)
    trajSet(link05, TRAP, 0.0, 110.0, 280.0)
    trajSet(link06, TRAP, 68.5, 90.0, 220.0)
    trajSet(link07, TRAP, 0.0, 120.0, 300.0)
}

wait(5.0)

trajClear()
wait(0.25)
stop()

# ========================================================
# END — Total runtime ~105s
#
# Comparison checklist:
#  [x] Phase 2 & 5 settle: residual oscillation amplitude
#  [x] Phase 3 tracking: peak & RMS position error
#  [x] Phase 4 energy: total mechanical energy over time
#  [x] Phase 6 reversals: overshoot at each direction change
#  [x] Overall: does the robot return to load pose at end?
# ========================================================
