# --------------------------------------------
# TEST 3 - Satellite Interception & Abort (Aggressive reversals)
# --------------------------------------------
#
# PURPOSE
# -------
#   This test stresses stability boundaries. Showing
#   how integrators behave near non-linear regions.
#   It also provides information on each methods
#   sensitivity to step size
#
# TORQUE MODE
# -----------
#   Controlled
#
# DISCRETE TIMES
# --------------
# (1/30), (1/60), (1/120), (1/240), (1/480), (1/960)
#
# JOINT LIMITS
# ------------
#   j1-j6:  +/- ~180 deg (+/-3.14149 rad)
#   v_max:  5.38 deg/s (0.0940 rad/s) hardware limit
#   t_max:  50 Nm per joint
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

wait(2.5)
trajClear()
wait(0.25)

# Begins sim run and logging.
start()

# Deployment of arm (Smooth approach)
parallel(20.0) {
    trajSet(link01, TRAP, -30.0, 3.0, 10.0)
    trajSet(link02, TRAP,  45.0, 3.0, 10.0)
    trajSet(link03, TRAP, -90.0, 3.0, 10.0)
    trajSet(link04, TRAP,   0.0, 3.0, 10.0)
    trajSet(link05, TRAP,  45.0, 3.0, 10.0)
    trajSet(link06, TRAP,   0.0, 3.0, 10.0)
}

wait(20.0)

# Tracking (Multisine and stabilisation test)
parallel(40.0) {
    trajSet(link01, MSINE, 40.0, -30.0, 20.0, 0.015, 0.0, 10.0, 0.025, 90.0)
    trajSet(link02, MSINE, 40.0,  45.0, 15.0, 0.015, 0.0,  8.0, 0.025, 45.0, 4.0, 0.040, 30.0)
    trajSet(link03, MSINE, 40.0, -90.0, 12.0, 0.018, 0.0,  7.0, 0.030, 120.0)
    trajSet(link04, MSINE, 40.0,   0.0, 12.0, 0.020, 0.0,  7.0, 0.032, 60.0, 4.0, 0.050, 150.0)
    trajSet(link05, MSINE, 40.0,  45.0, 10.0, 0.022, 0.0,  6.0, 0.036, 90.0, 3.5, 0.058, 0.0)
    trajSet(link06, MSINE, 40.0,   0.0,  8.0, 0.025, 0.0,  5.0, 0.040, 45.0)
}

wait(20.0)

# Emergency Abort (Stress Test within expected limit)
parallel(15.0) {
    trajSet(link01, TRAP,  60.0, 5.0, 80.0)
    trajSet(link02, TRAP, -60.0, 5.0, 80.0)
    trajSet(link03, TRAP,  60.0, 5.0, 80.0)
    trajSet(link04, TRAP, -60.0, 5.0, 80.0)
    trajSet(link05, TRAP, -60.0, 5.0, 80.0)
    trajSet(link06, TRAP,  60.0, 5.0, 80.0)
}

wait(20.0)
trajClear()
wait(0.25)

stop()
# --------------------------------------------
# END -> ~60 seconds
# --------------------------------------------