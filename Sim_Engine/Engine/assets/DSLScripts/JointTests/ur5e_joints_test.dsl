# ============================================================
# UR5e JOINT DAMPING IDENTIFICATION (PID)
# - Gravity OFF
# - One joint at a time
# - Two velocity regimes
#
# Joint limits (deg):
# J1: [-360, +360]
# J2: [-180, +180]
# J3: [-180, +180]
# J4: [-360, +360]
# J5: [-360, +360]
# J6: [-360, +360]
# J7: Fixed
# J8: Fixed
#
# Joint Omega limits (deg/s):
# J1-J6: [0, +180]
#
# Created by: SaltyJoss
# Assisted by: GitHub Copilot
# ============================================================

set(integrator, rk4)
load(robot, UR5e)
wait(2.0)

start()
wait(0.35)

# ------------------------------------------------------------
# Home / neutral pose (elbow-down, non-singular)
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0, 60.0, 120.0)
    trajSet(link02, TRAP, -90.0, 60.0, 120.0)
    trajSet(link03, TRAP,  90.0, 60.0, 120.0)
    trajSet(link04, TRAP,   0.0, 60.0, 120.0)
    trajSet(link05, TRAP,   0.0, 60.0, 120.0)
    trajSet(link06, TRAP,   0.0, 60.0, 120.0)
}

wait(3.0)
trajClear()
wait(0.25)

# ============================================================
# JOINT01 (Base)
# ============================================================

trajSet(link01, TRAP,  45.0, 30.0, 120.0)
wait(2.5)
trajSet(link01, TRAP, -45.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link01, TRAP,  45.0, 60.0, 240.0)
wait(2.0)
trajSet(link01, TRAP, -45.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT02 (Shoulder)
# ============================================================

trajSet(link02, TRAP, -60.0, 30.0, 120.0)
wait(2.5)
trajSet(link02, TRAP, -120.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link02, TRAP, -60.0, 60.0, 240.0)
wait(2.0)
trajSet(link02, TRAP, -120.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT03 (Elbow)
# ============================================================

trajSet(link03, TRAP,  60.0, 30.0, 120.0)
wait(2.5)
trajSet(link03, TRAP, 120.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link03, TRAP,  60.0, 60.0, 240.0)
wait(2.0)
trajSet(link03, TRAP, 120.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT04 (Wrist 1)
# ============================================================

trajSet(link04, TRAP,  45.0, 30.0, 120.0)
wait(2.5)
trajSet(link04, TRAP, -45.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link04, TRAP,  45.0, 60.0, 240.0)
wait(2.0)
trajSet(link04, TRAP, -45.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT05 (Wrist 2)
# ============================================================

trajSet(link05, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link05, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link05, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link05, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT06 (Wrist 3)
# ============================================================

trajSet(link06, TRAP,  60.0, 30.0, 120.0)
wait(2.5)
trajSet(link06, TRAP, -60.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link06, TRAP,  60.0, 60.0, 240.0)
wait(2.0)
trajSet(link06, TRAP, -60.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(0.25)

stop()