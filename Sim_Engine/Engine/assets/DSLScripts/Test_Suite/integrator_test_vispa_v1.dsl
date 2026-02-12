# ========================================================
# Integrator Comparison Test - Airbus VISPA (6-DOF)
# ========================================================
# Duration:  ~120s total
# Purpose:   The VISPA (Versatile Instrument for Solar
#            Pointing and Astrophysics) is an Airbus space-
#            rated manipulator designed for micro-gravity
#            operations. Zero damping and zero friction on
#            every joint means the system is purely inertial
#            - there is NO physical dissipation. This is the
#            hardest possible test for an integrator because
#            energy conservation errors accumulate without
#            bound.
#
#            The ISS Canadarm2 and ERA operate under similar
#            conditions: zero-g, zero-friction bearings, and
#            very low joint speeds (< 6 deg/s). This script
#            replicates that operational envelope while also
#            pushing beyond it to stress-test integrators.
# --------------------------------------------------------
# Phases:
#   1 (0-15s)     TRAP  - berthing configuration deployment
#   2 (15-20s)    HOLD  - post-deploy settle; no damping
#                         means ANY energy error persists
#   3 (20–55s)    SINE  - single-freq tracking at ISS-class
#                         speeds (non-commensurate freqs)
#   4 (55-60s)    HOLD  - mid-mission thermal soak pause;
#                         energy should be exactly conserved
#   5 (60-95s)    MSINE - multi-freq broadband excitation;
#                         the stress test for energy drift
#   6 (95-100s)   HOLD  - post-excitation settle; this is
#                         the critical energy conservation
#                         checkpoint
#   7 (100-115s)  TRAP  - fast EVA-assist reversals back
#                         to stow configuration
#   8 (115-120s)  HOLD  - final settle before stow
# --------------------------------------------------------
# Joint limits (VISPA, all revolute):
#   j1–j6  ±180° (±3.1415 rad)
#   Hardware max velocity: ~5.38 deg/s (0.0940 rad/s)
#   Effort: 50 Nm per joint
#   Damping: 0.0 on all joints (space-rated bearings)
#   Friction: 0.0 on all joints
#
# Stow pose: all joints at 0°
# --------------------------------------------------------
# Link masses (kg):
#   link00: 0.627   (base adapter)
#   link01: 2.328   (shoulder yaw)
#   link02: 3.995   (upper arm - heaviest, 0.8m reach)
#   link03: 2.328   (elbow)
#   link04: 3.157   (forearm - 0.65m reach)
#   link05: 2.695   (wrist roll)
#   link06: 0.924   (end-effector flange)
#
# KEY PROPERTY: 
# Zero damping + zero friction means this is
# a conservative mechanical system. Any integrator that does
# not preserve Hamiltonian structure will inject or remove
# energy. Over 120s this becomes clearly measurable.
#
# NOTE: All trajectory speeds are kept within the hardware
# velocity envelope (~5.38 deg/s) during ISS-class phases.
# Stress-test phases deliberately exceed hardware limits to
# test integrator robustness under controller saturation.
# ========================================================

load(robot, VISPA)
set(integrator, rk4)
wait(1.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# --------------------------------------------------------
# PHASE 1: Trapezoid deployment to berthing pose (0 - 15s)
#
# Simulates initial deployment from stowed configuration
# to an EVA-support working pose. Speeds are kept within
# the ISS operational envelope (~4 deg/s max). The heavy
# upper arm (link02, 3.995 kg at 0.8m) creates large
# centripetal coupling with the base joint — integrators
# that don't handle coupled inertia will show drift here.
#
# Shoulder joints move slowly (high inertia loads), wrist
# joints move faster (low inertia, fine positioning).
# --------------------------------------------------------

parallel(15.0) {
    trajSet(link01, TRAP, 60.0, 3.5, 4.0)
    trajSet(link02, TRAP, -45.0, 3.0, 3.5)
    trajSet(link03, TRAP, 30.0, 3.5, 4.0)
    trajSet(link04, TRAP, -90.0, 4.0, 5.0)
    trajSet(link05, TRAP, 45.0, 4.5, 5.5)
    trajSet(link06, TRAP, -30.0, 5.0, 6.0)
}

wait(15.0)

# --------------------------------------------------------
# PHASE 2: Post-deployment hold (15 - 20s)
#
# This is the first energy conservation checkpoint. With
# zero damping and zero friction, there is no physical
# mechanism to dissipate residual oscillation. Any motion
# seen here is purely numerical:
#  - Euler will show growing oscillations (energy gain)
#  - Midpoint/Heun show slow energy drift
#  - RK4 shows very small residual; RK45 near-zero
#
# ISS context: after Canadarm2 completes a traverse, the
# control system holds position and waits for thermal
# equilibration. Drift during hold = mission failure.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 3: Sinusoidal tracking — ISS inspection pass (20 - 55s)
#
# Simulates a slow-sweep visual inspection of an external
# module. Non-commensurate frequencies ensure the combined
# 6-DOF motion never repeats, preventing periodic error
# cancellation that would hide integrator deficiencies.
#
# Frequencies are chosen to be very low (0.03–0.09 Hz,
# periods 11–33s) matching real ISS operational cadence.
# Amplitudes are moderate (10–25°) to stay well inside
# joint limits while generating meaningful coupled torques.
#
# The 0.8m upper arm (link02) swinging at even low speed
# creates significant centripetal loading on the shoulder
# (link01) — this coupling is where single-rate methods
# struggle most.
# --------------------------------------------------------

parallel(35.0) {
    trajSet(link01, SINE, 35.0, 50.0, 20.0, 0.037)
    trajSet(link02, SINE, 35.0, -35.0, 18.0, 0.053)
    trajSet(link03, SINE, 35.0, 25.0, 15.0, 0.071)
    trajSet(link04, SINE, 35.0, -70.0, 25.0, 0.043)
    trajSet(link05, SINE, 35.0, 35.0, 22.0, 0.061)
    trajSet(link06, SINE, 35.0, -20.0, 18.0, 0.089)
}

wait(35.0)

# --------------------------------------------------------
# PHASE 4: Mid-mission thermal soak hold (55 - 60s)
#
# Second energy checkpoint. The system has been under
# continuous sinusoidal tracking for 35 seconds. With zero
# damping, the total mechanical energy at the end of Phase
# 3 should equal the energy at the start of Phase 3 to
# within integrator precision.
#
# ISS analogy: during external payload operations, the arm
# parks while astronauts reposition cameras or the station
# attitude adjusts. Any drift during this hold indicates
# the integrator is not preserving the Hamiltonian.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 5: Multisine broadband excitation (60 - 95s)
#
# The primary stress test. 2–3 overlapping sine components
# per joint with irrational frequency ratios. 35 seconds
# of sustained aperiodic motion creates the worst case for
# energy drift accumulation.
#
# The VISPA's long arm segments (0.8m + 0.65m) create
# large moment arms. Combined with the heavy upper arm
# (3.995 kg) and zero damping, even small per-step energy
# errors compound rapidly. After 35s of multisine at
# dt=1/180, that's ~6300 integration steps — enough for
# Euler to show visible energy growth.
#
# Frequency ratios are deliberately irrational (no pair
# shares a simple rational relationship) to guarantee
# the trajectory never repeats within the 35s window.
#
# Component amplitudes are conservative (6–18°) so the
# sum of all components stays within ±45° of centre,
# keeping well inside the ±180° joint limits.
# --------------------------------------------------------

parallel(35.0) {
    trajSet(link01, MSINE, 35.0, 20.0,    12.0, 0.041, 0.0,    6.0, 0.113, 90.0)
    trajSet(link02, MSINE, 35.0, -30.0,   14.0, 0.053, 0.0,    8.0, 0.127, 45.0,    4.0, 0.239, 30.0)
    trajSet(link03, MSINE, 35.0, 15.0,    10.0, 0.067, 0.0,    5.0, 0.149, 120.0)
    trajSet(link04, MSINE, 35.0, -50.0,   18.0, 0.047, 0.0,    10.0, 0.137, 60.0,    5.0, 0.251, 150.0)
    trajSet(link05, MSINE, 35.0, 20.0,    14.0, 0.059, 0.0,    7.0, 0.163, 90.0,    3.0, 0.277, 0.0)
    trajSet(link06, MSINE, 35.0, -10.0,   12.0, 0.073, 0.0,    6.0, 0.179, 45.0)
}
wait(35.0)

# --------------------------------------------------------
# PHASE 6: Post-excitation settle (95 - 100s)
#
# Third and most critical energy checkpoint. After 70
# seconds of continuous motion (Phases 3+5) with zero
# physical dissipation, any residual velocity at this
# hold is integrator-injected energy.
#
# Expected behaviour:
#  - Euler:   visible oscillation, growing amplitude
#  - Midpoint: small oscillation, slowly growing
#  - Heun:    small oscillation, stable or slow drift
#  - RK4:     near-zero residual, stable
#  - RK45:    negligible residual (adaptive step)
#
# The undamped VISPA acts as an energy amplifier: any
# numerical energy injection has nowhere to go and
# compounds with each subsequent step.
# --------------------------------------------------------

wait(5.0)

# --------------------------------------------------------
# PHASE 7: Fast EVA-assist reversals (100 - 115s)
#
# Three rapid back-and-forth sweeps with increasing
# aggressiveness, simulating emergency repositioning
# during EVA operations. These intentionally exceed the
# ISS hardware velocity envelope to test how each
# integrator handles controller saturation and torque
# clamping in a zero-damping system.
#
# Reversal points create velocity discontinuities where
# integrators diverge most visibly. The zero-friction
# bearings mean there is no natural braking — the
# controller must do ALL the work, and any integration
# error passes directly through to the joints.
# --------------------------------------------------------

# 7a: outward reposition - moderate speed
parallel(5.0) {
    trajSet(link01, TRAP, -40.0, 25.0, 55.0)
    trajSet(link02, TRAP, 30.0, 20.0, 45.0)
    trajSet(link03, TRAP, -35.0, 25.0, 55.0)
    trajSet(link04, TRAP, 20.0, 30.0, 65.0)
    trajSet(link05, TRAP, -30.0, 35.0, 75.0)
    trajSet(link06, TRAP, 40.0, 40.0, 85.0)
}

wait(5.0)

# 7b: snap through centre - aggressive
parallel(5.0) {
    trajSet(link01, TRAP, 50.0, 40.0, 90.0)
    trajSet(link02, TRAP, -50.0, 35.0, 80.0)
    trajSet(link03, TRAP, 45.0, 40.0, 90.0)
    trajSet(link04, TRAP, -60.0, 45.0, 100.0)
    trajSet(link05, TRAP, 50.0, 50.0, 110.0)
    trajSet(link06, TRAP, -45.0, 55.0, 120.0)
}

wait(5.0)

# 7c: return to stow (0° all joints) - highest accel
parallel(5.0) {
    trajSet(link01, TRAP, 0.0, 55.0, 130.0)
    trajSet(link02, TRAP, 0.0, 50.0, 120.0)
    trajSet(link03, TRAP, 0.0, 55.0, 130.0)
    trajSet(link04, TRAP, 0.0, 60.0, 140.0)
    trajSet(link05, TRAP, 0.0, 65.0, 150.0)
    trajSet(link06, TRAP, 0.0, 70.0, 160.0)
}

wait(5.0)

# --------------------------------------------------------
# PHASE 8: Final stow hold (115 – 120s)
#
# The arm should be back at stow pose (all joints 0°).
# Final energy and position check. With zero damping any
# accumulated energy error from the entire 120s run will
# be visible as residual oscillation about the stow pose.
# --------------------------------------------------------

wait(5.0)

trajClear()
wait(0.25)

# save(plots, VISPA_integrator_test) -> TBA
# save(data, VISPA_integrator_test)  -> TBA

stop()

# ========================================================
# END - Total runtime ~120s
#
# What makes VISPA uniquely revealing for integrators:
#
#  1. ZERO DAMPING, ZERO FRICTION: Unlike the Panda (up to
#     19.0 damping) or iiwa14 (moderate damping), the VISPA
#     has absolutely no physical energy dissipation. Every
#     numerical energy error accumulates permanently. This
#     is the purest test of integrator energy conservation.
#
#  2. LONG MOMENT ARMS: The 0.8m upper arm (link02) and
#     0.65m forearm (link04) create large centripetal and
#     Coriolis coupling forces. These are exactly the terms
#     that cause Euler to inject energy via linear approx-
#     imation of the curved phase-space trajectory.
#
#  3. MASS ASYMMETRY: 6.3:1 ratio between the heaviest
#     link (link02, 3.995 kg) and lightest (link00, 0.627
#     kg). Large inertia ratios amplify numerical coupling
#     errors in the mass matrix factorisation.
#
#  4. SPACE-RATE VELOCITIES: Phases 1-4 operate at ISS-
#     class speeds (< 5.4 deg/s). This is where energy
#     drift is subtle but operationally critical. Phase 7
#     deliberately exceeds hardware limits to test satur-
#     ation handling.
#
#  5. 8-PHASE STRUCTURE with 3 hold checkpoints: energy
#     can be measured at t=20s, t=60s, and t=100s. The
#     growth rate between checkpoints reveals whether the
#     integrator's energy error scales linearly (Euler),
#     quadratically (midpoint), or higher-order (RK4/45).
#
# Comparison checklist:
#  [ ] Phase 2 (t=20s): residual velocity on all joints
#  [ ] Phase 3: RMS tracking error per joint
#  [ ] Phase 4 (t=60s): total energy vs Phase 2 energy
#  [ ] Phase 5: energy drift rate over 35s of multisine
#  [ ] Phase 6 (t=100s): energy vs Phase 4 (should match)
#  [ ] Phase 7: overshoot at each reversal point
#  [ ] Phase 8: final deviation from stow pose (0° all)
#  [ ] Overall: does energy grow (Euler), stay flat (RK4),
#              or adapt (RK45) across the full 120s?
# ========================================================
