# ============================================================
# IIWA14 JOINT DAMPING IDENTIFICATION (PID)
# - Gravity OFF
# - One joint at a time
# - Two velocity regimes
#
# Joint limits (deg):
# J1: [-170, +170]
# J2: [-120, +120]
# J3: [-170, +170]
# J4: [-120, +120]
# J5: [-170, +170]
# J6: [-120, +120]
# J7: [-175, +175]
#
# Joint Omega limits (deg/s):
# J1-J7: [0, +180]
#
# Created by: SaltyJoss
# Double Checked using GitHub Copilot
# ============================================================

set(integrator, rk4)
load(robot, iiwa14)
wait(2.0)

start()
wait(0.35)

# ------------------------------------------------------------
# Home / neutral pose (your load_pose, non-singular)
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0, 60.0, 120.0)
    trajSet(link02, TRAP,   0.4, 60.0, 120.0)
    trajSet(link03, TRAP,   0.0, 60.0, 120.0)
    trajSet(link04, TRAP, -68.5, 60.0, 120.0)
    trajSet(link05, TRAP,   0.0, 60.0, 120.0)
    trajSet(link06, TRAP,  68.5, 60.0, 120.0)
    trajSet(link07, TRAP,   0.0, 60.0, 120.0)
}

wait(3.0)
trajClear()
wait(0.25)

# ============================================================
# JOINT 1 (Base)
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
# JOINT 2 (Shoulder)
# ============================================================

trajSet(link02, TRAP, -30.0, 30.0, 120.0)
wait(2.5)
trajSet(link02, TRAP, -90.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link02, TRAP, -30.0, 60.0, 240.0)
wait(2.0)
trajSet(link02, TRAP, -90.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT 3 (Elbow)
# ============================================================

trajSet(link03, TRAP,  45.0, 30.0, 120.0)
wait(2.5)
trajSet(link03, TRAP, -45.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link03, TRAP,  45.0, 60.0, 240.0)
wait(2.0)
trajSet(link03, TRAP, -45.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT 4
# ============================================================

trajSet(link04, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link04, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link04, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link04, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT 5
# ============================================================

trajSet(link05, TRAP,  45.0, 30.0, 120.0)
wait(2.5)
trajSet(link05, TRAP, -45.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link05, TRAP,  45.0, 60.0, 240.0)
wait(2.0)
trajSet(link05, TRAP, -45.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT 6
# ============================================================

trajSet(link06, TRAP,  30.0, 30.0, 120.0)
wait(2.5)
trajSet(link06, TRAP, -30.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link06, TRAP,  30.0, 60.0, 240.0)
wait(2.0)
trajSet(link06, TRAP, -30.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(5.0)

# ============================================================
# JOINT 7 (Tool)
# ============================================================

trajSet(link07, TRAP,  45.0, 30.0, 120.0)
wait(2.5)
trajSet(link07, TRAP, -45.0, 30.0, 120.0)
wait(2.5)

trajClear()
wait(0.25)

trajSet(link07, TRAP,  45.0, 60.0, 240.0)
wait(2.0)
trajSet(link07, TRAP, -45.0, 60.0, 240.0)
wait(2.0)

trajClear()
wait(0.25)

stop()