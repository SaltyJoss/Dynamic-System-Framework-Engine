# --------------------------------------------
# MISSION: Airbus VISPA => Satellite Interception & Abort (Aggressive reversals)
# --------------------------------------------
#
# MISSION PROFILE
# ---------------
#   1. System Initialization & Calibration Check (0.0s - 5.0s)
#   2. Nominal Deployment to Ready-State Observer Position (5.0s - 35.0s)
#   3. Proximity Close-Approach Synchronisation (35.0s - 95.0s)
#   4. Multi-Point Surface Inspection (95.0s - 215.0s)
#   5. Synchronised Contact & Stabilisation Hold (215.0s - 275.0s)
#   6. Secure Safe-State Stow & Mission Completion (275.0s - 335.0s)
#
# JOINT LIMITS
# ------------
#   j1-j6:  +/- ~180 deg (+/-3.14149 rad)
#   v_max:  5.38 deg/s (0.0940 rad/s) hardware limit
#   v_operating: < 1.5 deg/s
#   Q_max:  50 Nm per joint
#   Damping / Friction: 0.2 / 0.05 (all joints)
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
# --------------------------------------------

load(robot, VISPA)

wait(4.0)
trajClear()
wait(1.0)

# Begins sim run and logging.
start()

# Nominal Deployment
parallel(30.0) {
    trajSet(link01, TRAP, -30.0, 1.5, 5.0)
    trajSet(link02, TRAP,  45.0, 2.0, 5.0)
    trajSet(link03, TRAP, -90.0, 2.0, 5.0)
    trajSet(link04, TRAP,   0.0, 1.0, 3.0)
    trajSet(link05, TRAP,  45.0, 1.5, 4.0)
    trajSet(link06, TRAP,   0.0, 1.0, 3.0)
}

wait(30.0)

# Close-approach synchronisation
parallel(60.0) {
    trajSet(link01, TRAP, -45.0, 0.5, 10.0)
    trajSet(link02, TRAP,  55.0, 0.4, 10.0)
    trajSet(link03, TRAP, -80.0, 0.4, 10.0)
    trajSet(link04, TRAP,  10.0, 0.3, 8.0)
    trajSet(link05, TRAP,  30.0, 0.5, 10.0)
    trajSet(link06, TRAP,  15.0, 0.5, 8.0)
}

wait(60.0)

# Multipoint surface inspection
parallel(120.0) {
    trajSet(link01, MSINE, 120.0, -45.0, 4.0, 0.002, 0.0, 2.0, 0.004, 90.0)
    trajSet(link02, MSINE, 120.0,  55.0, 3.0, 0.001, 0.0, 1.5, 0.003, 45.0)
    trajSet(link03, MSINE, 120.0, -80.0, 3.0, 0.002, 0.0, 1.0, 0.004, 120.0)
    trajSet(link04, MSINE, 120.0,  10.0, 2.5, 0.003, 0.0, 1.0, 0.005, 60.0)
    trajSet(link05, MSINE, 120.0,  30.0, 2.0, 0.004, 0.0, 1.0, 0.006, 90.0)
    trajSet(link06, MSINE, 120.0,  15.0, 2.0, 0.005, 0.0, 1.0, 0.007, 45.0)
}

wait(120.0)

# Contact, docking, and servicing lock
parallel(60.0) {
    trajSet(link01, TRAP,  -20.0, 0.8, 8.0)
    trajSet(link02, TRAP,   30.0, 0.8, 8.0)
    trajSet(link03, TRAP, -100.0, 0.8, 8.0)
    trajSet(link04, TRAP,    0.0, 0.5, 8.0)
    trajSet(link05, TRAP,   60.0, 0.8, 8.0)
    trajSet(link06, TRAP,    0.0, 0.5, 8.0)
}

wait(60.0)

# Safe-state return
parallel(60.0) {
    trajSet(link01, TRAP, 0.0, 1.0, 10.0)
    trajSet(link02, TRAP, 0.0, 1.0, 10.0)
    trajSet(link03, TRAP, 0.0, 1.0, 10.0)
    trajSet(link04, TRAP, 0.0, 1.0, 10.0)
    trajSet(link05, TRAP, 0.0, 1.0, 10.0)
    trajSet(link06, TRAP, 0.0, 1.0, 10.0)
}

wait(60.0)

trajClear()
wait(1.0)

stop()
# --------------------------------------------
# END -> ~60 seconds
# --------------------------------------------