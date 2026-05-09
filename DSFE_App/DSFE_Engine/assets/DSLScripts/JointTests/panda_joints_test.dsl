# ============================================================
# PANDA JOINT DAMPING IDENTIFICATION (PID)
# - Gravity OFF
# - One joint at a time
# - Two velocity regimes
#
# Joint limits (deg):
# J1: [-166, +166]
# J2: [-101, +101]
# J3: [-166, +166]
# J4: [-176,   -4]
# J5: [-166, +166]
# J6: [  -1, +215]
# J7: [-166, +166]
#
# Joint Omega limits (deg/s):
# J1-J7: [0, +180]
#
# Created by: SaltyJoss
# Double Checked using GitHub Copilot
# ============================================================

set(integrator, rk4)
load(robot, Panda)
wait(2.0)

start()
wait(0.35)

# ------------------------------------------------------------
# Home / neutral pose (inside limits, non-singular)
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0, 60.0, 120.0)
    trajSet(link02, TRAP, -45.0, 60.0, 120.0)
    trajSet(link03, TRAP,   0.0, 60.0, 120.0)
    trajSet(link04, TRAP, -90.0, 60.0, 120.0)
    trajSet(link05, TRAP,   0.0, 60.0, 120.0)
    trajSet(link06, TRAP,  90.0, 60.0, 120.0)
    trajSet(link07, TRAP,   0.0, 60.0, 120.0)
}

wait(3.0)
trajClear()
wait(0.25)

# ============================================================
# JOINT01
# ============================================================

trajSet(link01, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link01, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link01, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link01, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT02
# ============================================================

trajSet(link02, TRAP, -15.0, 30.0, 120.0)
wait(2.5)
trajSet(link02, TRAP, -75.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link02, TRAP, -15.0, 60.0, 240.0)
wait(2.0)
trajSet(link02, TRAP, -75.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT03
# ============================================================

trajSet(link03, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link03, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link03, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link03, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT04 (asymmetric only)
# ============================================================

trajSet(link04, TRAP, -45.0, 30.0, 120.0)
wait(2.5)
trajSet(link04, TRAP, -135.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link04, TRAP, -45.0, 60.0, 240.0)
wait(2.0)
trajSet(link04, TRAP, -135.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT05
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
# JOINT06 (positive only)
# ============================================================

trajSet(link06, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link06, TRAP, 120.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link06, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link06, TRAP, 120.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT07
# ============================================================

trajSet(link07, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link07, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link07, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link07, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(0.25)

stop()
