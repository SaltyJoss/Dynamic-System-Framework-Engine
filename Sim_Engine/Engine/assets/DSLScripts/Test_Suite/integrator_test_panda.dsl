# ========================================================
# Integrator Comparison Test — Franka Panda (7-DOF)
# ========================================================
# Duration:  ~100s total
# Purpose:   The Panda's extreme damping variation (19.0 at
#            base, 1.5 at wrist) makes it ideal for exposing
#            integrator differences in stiff vs. compliant
#            regimes within the same kinematic chain.
# --------------------------------------------------------
# Phases:
#   1 (0–10s)    TRAP  - sweep to working configuration
#   2 (10–15s)   HOLD  - settle; check post-TRAP drift
#   3 (15–45s)   SINE  - single-freq tracking (non-commensurate)
#   4 (45–80s)   MSINE - multi-freq broadband excitation
#   5 (80–85s)   HOLD  - settle; check energy decay
#   6 (85–100s)  TRAP  - fast reversals back to load pose
# --------------------------------------------------------
# Joint limits (Panda):
#   j1,3,5,7  ±166°     
#   j2  ±101°
#   j4  -176° to -4°    
#   j6  -1° to 215°
#
# Load pose: (0, -45, 0, -135, 0, 90, 0)°
# --------------------------------------------------------
# NOTE: j4 and j6 have asymmetric ranges. All centres
# and amplitudes below are chosen to stay 10° inside
# the hard limits at all times.
# ========================================================

load(robot, panda)
set(integrator, rk4)
wait(1.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# --------------------------------------------------------
# PHASE 1: Trapezoid to working pose (0 – 10s)
# Move every joint away from load pose so the subsequent
# phases have room to oscillate in both directions.
# Moderate speed on the high-damping base joints,
# faster on the low-damping wrist joints.
# --------------------------------------------------------

parallel(10.0)
{
    trajSet(link01, TRAP, 45.0, 35.0, 70.0)
    trajSet(link02, TRAP, -20.0, 30.0, 60.0)
    trajSet(link03, TRAP, 30.0, 35.0, 70.0)
    trajSet(link04, TRAP, -90.0, 30.0, 60.0)
    trajSet(link05, TRAP, 25.0, 50.0, 100.0)
    trajSet(link06, TRAP, 120.0, 45.0, 90.0)
    trajSet(link07, TRAP, 20.0, 55.0, 110.0)
}

wait(10.0)

# --------------------------------------------------------
# PHASE 2: Post-TRAP settle (10 – 15s)
# Hold position. Integrators with poor damping handling
# will show residual oscillation — especially on joints
# 5-7 where physical damping is very low (1.5-2.0).
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 3: Sinusoidal tracking (15 – 45s)
# Non-commensurate frequencies so combined motion never
# repeats. The 13x damping ratio between j1 (19.0) and
# j7 (1.5) means different joints need different
# numerical treatment — this is where single-rate
# integrators struggle.
# --------------------------------------------------------

parallel(30.0)
{
    trajSet(link01, SINE, 30.0, 22.0, 20.0, 0.11)
    trajSet(link02, SINE, 30.0, -30.0, 18.0, 0.17)
    trajSet(link03, SINE, 30.0, 15.0, 22.0, 0.23)
    trajSet(link04, SINE, 30.0, -90.0, 35.0, 0.29)
    trajSet(link05, SINE, 30.0, 12.0, 20.0, 0.37)
    trajSet(link06, SINE, 30.0, 100.0, 30.0, 0.41)
    trajSet(link07, SINE, 30.0, 10.0, 25.0, 0.59)
}

wait(30.0)

# --------------------------------------------------------
# PHASE 4: Multisine excitation (45 – 80s)
# The stress test. 2-3 overlapping sine components per
# joint with irrational frequency ratios. 35 seconds of
# sustained complex motion — long enough for energy drift
# and numerical damping to become clearly measurable.
#
# j4 and j6 use centres deep inside their asymmetric
# ranges with conservative amplitudes.
# --------------------------------------------------------

parallel(35.0)
{
    trajSet(link01, MSINE, 35.0, 10.0,    15.0, 0.10, 0.0,   8.0, 0.29, 90.0)
    trajSet(link02, MSINE, 35.0, -25.0,   14.0, 0.13, 0.0,   7.0, 0.37, 45.0)
    trajSet(link03, MSINE, 35.0, 8.0,     16.0, 0.16, 0.0,   6.0, 0.43, 120.0,   3.0, 0.79, 30.0)
    trajSet(link04, MSINE, 35.0, -90.0,   20.0, 0.18, 0.0,   12.0, 0.41, 60.0,   5.0, 0.73, 150.0)
    trajSet(link05, MSINE, 35.0, 5.0,     14.0, 0.22, 0.0,   9.0, 0.53, 90.0)
    trajSet(link06, MSINE, 35.0, 90.0,    18.0, 0.26, 0.0,   10.0, 0.61, 45.0,   5.0, 0.97, 0.0)
    trajSet(link07, MSINE, 35.0, 5.0,     18.0, 0.31, 0.0,   8.0, 0.71, 60.0)
}

wait(35.0)

# --------------------------------------------------------
# PHASE 5: Post-multisine settle (80 – 85s)
# Hold after the most aggressive phase. Total mechanical
# energy should decay smoothly to zero. Euler will show
# energy growth; higher-order methods show clean decay.
# The low-damping wrist joints (j5-j7) are the canary —
# if an integrator injects energy, it shows here first.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 6: Fast opposing trapezoids (85 – 100s)
# Three rapid sweeps with increasing aggressiveness,
# ending at load pose. Tests velocity discontinuities
# and controller saturation handling.
# --------------------------------------------------------

# 6a: aggressive outward sweep
parallel(5.0)
{
    trajSet(link01, TRAP, -40.0, 70.0, 150.0)
    trajSet(link02, TRAP, -60.0, 55.0, 120.0)
    trajSet(link03, TRAP, -35.0, 65.0, 140.0)
    trajSet(link04, TRAP, -50.0, 60.0, 130.0)
    trajSet(link05, TRAP, -30.0, 80.0, 170.0)
    trajSet(link06, TRAP, 60.0, 70.0, 150.0)
    trajSet(link07, TRAP, -25.0, 90.0, 190.0)
}

wait(5.0)

# 6b: snap back through centre
parallel(5.0)
{
    trajSet(link01, TRAP, 50.0, 80.0, 180.0)
    trajSet(link02, TRAP, 10.0, 65.0, 150.0)
    trajSet(link03, TRAP, 40.0, 75.0, 170.0)
    trajSet(link04, TRAP, -130.0, 70.0, 160.0)
    trajSet(link05, TRAP, 35.0, 90.0, 200.0)
    trajSet(link06, TRAP, 140.0, 80.0, 180.0)
    trajSet(link07, TRAP, 30.0, 100.0, 220.0)
}

wait(5.0)

# 6c: return to load pose (highest accel)
parallel(5.0)
{
    trajSet(link01, TRAP, 0.0, 90.0, 220.0)
    trajSet(link02, TRAP, -45.0, 75.0, 180.0)
    trajSet(link03, TRAP, 0.0, 85.0, 210.0)
    trajSet(link04, TRAP, -135.0, 80.0, 200.0)
    trajSet(link05, TRAP, 0.0, 100.0, 250.0)
    trajSet(link06, TRAP, 90.0, 85.0, 200.0)
    trajSet(link07, TRAP, 0.0, 110.0, 270.0)
}

wait(5.0)

trajClear()
wait(0.25)
stop()

# ========================================================
# END — Total runtime ~100s
#
# What makes the Panda uniquely revealing for integrators:
#
#  1. DAMPING SPREAD: 13:1 ratio between j1 and j7 means
#     the system is mildly stiff. Euler will inject energy
#     into the low-damping wrist joints while overdamping
#     the base. Higher-order methods handle both regimes.
#
#  2. ASYMMETRIC LIMITS: j4 (-176° to -4°) and j6 (-1° to
#     215°) operate in offset ranges. Barrier/clamping
#     forces at limits interact with the integrator.
#
#  3. MASS DISTRIBUTION: Heavy elbow (3.59kg) on a light
#     wrist (0.74kg) creates large inertia ratios that
#     amplify numerical errors in the coupled dynamics.
#
# Comparison checklist:
#  [ ] Phase 2 & 5: residual oscillation on j5/j6/j7
#  [ ] Phase 3: RMS tracking error per joint
#  [ ] Phase 4: total energy drift over 35s
#  [ ] Phase 6: overshoot at each reversal
#  [ ] Final: deviation from load pose after stop
# ========================================================
