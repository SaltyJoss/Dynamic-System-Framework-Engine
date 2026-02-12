# ================================================================
# INTEGRATOR VALIDATION SUITE - Airbus VISPA (6-DOF)
# ================================================================
# Duration:  ~120s  |  10 phases
#
# PURPOSE
# -------
# To demonstrate that numerical integrator choice has direct,
# measurable consequences for space manipulator operations.
# The VISPA's zero-damping, zero-friction joints create a
# perfectly conservative mechanical system.
# A very harsh environment means there is NO physical 
# dissipation to mask any integration errors.
#
# OPERATIONAL CONTEXT
# -------------------
# This script replicates the ISS operational envelope in
# Phases 1-4 and then deliberately exceeds it in Phases
# 5 and 7-8 to amplify integrator differences into clearly
# visible, quantifiable metrics.
#
# WHY VISPA
# ---------------------------------
#  * Zero damping / Zero friction - conservative system;
#  * Long moment arms - 0.8m upper arm (link02) + 0.65m
#    forearm (link04) create large centripetal and Coriolis
#    coupling.
#  * 6.3:1 mass ratio (link02 -> 3.995 kg vs link00 -> 0.627 kg)
#    creates a stiff inertia coupling in the mass matrix.
#  * ISS-class velocity limits (5.38 deg/s hardware cap)
#    mean the arm operates in a slow, high-precision manor,
#    where small energy drift matters most.
#
# PHASE STRUCTURE
# ---------------
#   Phase  Time(s)       Type
#   -----  ----------    ------
#    1       0 -  15s    TRAP
#    2      15 -  20s    HOLD
#    3      20 -  55s    SINE
#    4      55 -  60s    HOLD
#    5      60 -  95s    MSINE
#    6      95 - 100s    HOLD
#    7     100 - 105s    TRAP
#    8     105 - 113s    TRAP
#    9     113 - 117s    TRAP
#   10     117 - 120s    HOLD
#
# JOINT LIMITS
# ------------
#   j1-j6:  +/-180 deg (+/-3.1415 rad)
#   v_max:  5.38 deg/s (0.0940 rad/s) hardware limit
#   t_max:  50 Nm per joint
#   Damping / Friction: 0.0 / 0.0 (all joints)
#
# LINK MASSES (kg)
# ----------------
#   link00  0.627   base adapter (gold coloured)
#   link01  2.328   shoulder yaw
#   link02  3.995   upper arm (0.8m, heaviest link)
#   link03  2.328   elbow
#   link04  3.157   forearm (0.65m)
#   link05  2.695   wrist roll
#   link06  0.924   end-effector flange
#
# Create By: Joss Salton
# GitHub:    SaltyJoss
#
# DISCLAIMER:
# -----
# Cross-Checked before release using GitHub Copilot.
# ================================================================

load(robot, VISPA)
set(integrator, rk4)
wait(1.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# ----------------------------------------------------------------
# PHASE 1: Deploy to berthing pose (0 - 15s)
# ----------------------------------------------------------------

parallel(15.0) {
    trajSet(link01, TRAP, 60.0, 3.5, 4.0)
    trajSet(link02, TRAP, -45.0, 3.0, 3.5)
    trajSet(link03, TRAP, 30.0, 3.5, 4.0)
    trajSet(link04, TRAP, -90.0, 4.0, 5.0)
    trajSet(link05, TRAP, 45.0, 4.5, 5.5)
    trajSet(link06, TRAP, -30.0, 5.0, 6.0)
}

wait(15.0)

# ----------------------------------------------------------------
# PHASE 2: Hold - Checkpoint A (15 - 20s)
# ----------------------------------------------------------------

wait(5.0)

# ----------------------------------------------------------------
# PHASE 3: Sinusoidal tracking - inspection sweep (20 - 55s)
# ----------------------------------------------------------------

parallel(35.0) {
    trajSet(link01, SINE, 35.0, 50.0, 20.0, 0.037, 0.0)
    trajSet(link02, SINE, 35.0, -35.0, 18.0, 0.053, 137.5)
    trajSet(link03, SINE, 35.0, 25.0, 15.0, 0.071, 275.0)
    trajSet(link04, SINE, 35.0, -70.0, 22.0, 0.043, 52.5)
    trajSet(link05, SINE, 35.0, 35.0, 20.0, 0.061, 190.0)
    trajSet(link06, SINE, 35.0, -20.0, 16.0, 0.083, 327.5)
}

wait(35.0)

# ----------------------------------------------------------------
# PHASE 4: Hold - Checkpoint B (55 - 60s)
# ----------------------------------------------------------------

wait(5.0)

# ----------------------------------------------------------------
# PHASE 5: Multisine broadband excitation (60 - 95s)
# ----------------------------------------------------------------

parallel(35.0) {
    trajSet(link01, MSINE, 35.0, 30.0,    14.0, 0.041, 0.0,     7.0, 0.066, 90.0)
    trajSet(link02, MSINE, 35.0, -25.0,   16.0, 0.047, 0.0,     9.0, 0.076, 45.0,     4.0, 0.123, 30.0)
    trajSet(link03, MSINE, 35.0, 15.0,    12.0, 0.053, 0.0,     6.0, 0.086, 120.0)
    trajSet(link04, MSINE, 35.0, -55.0,   16.0, 0.059, 0.0,    10.0, 0.095, 60.0,     5.0, 0.154, 150.0)
    trajSet(link05, MSINE, 35.0, 25.0,    14.0, 0.067, 0.0,     8.0, 0.108, 90.0,     4.0, 0.175, 0.0)
    trajSet(link06, MSINE, 35.0, -10.0,   12.0, 0.073, 0.0,     6.0, 0.118, 45.0)
}

wait(35.0)

# ----------------------------------------------------------------
# PHASE 6: Hold - Checkpoint C (95 - 100s)
# ----------------------------------------------------------------

wait(5.0) 

# ----------------------------------------------------------------
# PHASE 7: Moderate-speed reversals (100 - 105s)
# ----------------------------------------------------------------

parallel(5.0) {
    trajSet(link01, TRAP, -10.0, 12.0, 28.0)
    trajSet(link02, TRAP, 10.0, 10.0, 24.0)
    trajSet(link03, TRAP, -15.0, 12.0, 28.0)
    trajSet(link04, TRAP, -40.0, 14.0, 32.0)
    trajSet(link05, TRAP, 10.0, 15.0, 36.0)
    trajSet(link06, TRAP, 15.0, 16.0, 38.0)
}

wait(5.0)

# ----------------------------------------------------------------
# PHASE 8: Aggressive reversals (105 - 113s)
# ----------------------------------------------------------------

# 8a: aggressive outward sweep
parallel(4.0) {
    trajSet(link01, TRAP, -50.0, 35.0, 80.0)
    trajSet(link02, TRAP, 40.0, 30.0, 70.0)
    trajSet(link03, TRAP, -45.0, 35.0, 80.0)
    trajSet(link04, TRAP, 30.0, 40.0, 95.0)
    trajSet(link05, TRAP, -40.0, 45.0, 100.0)
    trajSet(link06, TRAP, 50.0, 50.0, 110.0)
}

wait(4.0)

# 8b: snap back through centre
parallel(4.0) {
    trajSet(link01, TRAP, 55.0, 45.0, 100.0)
    trajSet(link02, TRAP, -45.0, 40.0, 90.0)
    trajSet(link03, TRAP, 50.0, 45.0, 100.0)
    trajSet(link04, TRAP, -65.0, 50.0, 115.0)
    trajSet(link05, TRAP, 55.0, 55.0, 125.0)
    trajSet(link06, TRAP, -50.0, 55.0, 130.0)
}

wait(4.0)

# ----------------------------------------------------------------
# PHASE 9: Precision return to berthing pose (113 - 117s)
# ----------------------------------------------------------------

parallel(4.0) {
    trajSet(link01, TRAP, 60.0, 50.0, 120.0)
    trajSet(link02, TRAP, -45.0, 45.0, 110.0)
    trajSet(link03, TRAP, 30.0, 50.0, 120.0)
    trajSet(link04, TRAP, -90.0, 55.0, 130.0)
    trajSet(link05, TRAP, 45.0, 60.0, 140.0)
    trajSet(link06, TRAP, -30.0, 65.0, 150.0)
}

wait(4.0)

# ----------------------------------------------------------------
# PHASE 10: Hold - Checkpoint D (117 - 120s)
# ----------------------------------------------------------------

wait(3.0)

trajClear()
wait(0.25)

stop()

# ================================================================
# END - Total runtime: ~120s | 10 phases
# ================================================================
