# ============================================================
# INTEGRATOR EVALUATION + SHOWCASE SCRIPT (PANDA)
#  - Designed to stay strictly inside joint limits
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
# Assisted by: GitHub Copilot
# ============================================================

load(robot, panda)
set(integrator, rk4)
wait(2)

trajClear()
wait(0.25)

start()
wait(0.35)

# ------------------------------------------------------------
# Phase 0: Neutral centre pose
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  60.0, 120.0)
	trajSet(link02, TRAP,   0.0,  50.0, 100.0)
	trajSet(link03, TRAP,   0.0,  50.0, 100.0)
	trajSet(link04, TRAP, -90.0,  60.0, 120.0)
	trajSet(link05, TRAP,   0.0,  70.0, 140.0)
	trajSet(link06, TRAP,  90.0,  70.0, 140.0)
	trajSet(link07, TRAP,   0.0,  70.0, 140.0)
}
wait(2.4)
wait(0.30)

# ------------------------------------------------------------
# Phase 1: Low-frequency whole-arm breathing
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, SINE, 8.0,  0.0, 3.0, 0.10)
	trajSet(link02, SINE, 8.0,  0.0, 4.0, 0.12)
	trajSet(link03, SINE, 8.0,  0.0, 4.0, 0.12)
	trajSet(link04, SINE, 8.0, -90.0, 4.0, 0.15)
	trajSet(link05, SINE, 8.0,  0.0, 4.0, 0.18)
	trajSet(link06, SINE, 8.0, 90.0, 5.0, 0.20)
	trajSet(link07, SINE, 8.0,  0.0, 5.0, 0.20)
}
wait(8.0)
wait(0.30)

# ------------------------------------------------------------
# Phase 2: Re-centre (baseline reset)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  70.0, 140.0)
	trajSet(link02, TRAP,   0.0,  60.0, 120.0)
	trajSet(link03, TRAP,   0.0,  60.0, 120.0)
	trajSet(link04, TRAP, -90.0,  70.0, 140.0)
	trajSet(link05, TRAP,   0.0,  80.0, 160.0)
	trajSet(link06, TRAP,  90.0,  80.0, 160.0)
	trajSet(link07, TRAP,   0.0,  80.0, 160.0)
}
wait(2.2)
wait(0.25)

# ------------------------------------------------------------
# Phase 3: Moderate broadband excitation
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, SINE, 10.0,  0.0, 3.0, 0.15)
	trajSet(link02, SINE, 10.0,  0.0, 4.0, 0.16)
	trajSet(link03, SINE, 10.0,  0.0, 4.0, 0.16)
	trajSet(link04, MSINE, 10.0, -90.0, 3.0, 0.25,  0.0, 2.0, 0.50, 120.0)
	trajSet(link05, MSINE, 10.0,  0.0, 3.0, 0.30, 30.0, 3.0, 0.60, 160.0)
	trajSet(link06, MSINE, 10.0, 90.0, 4.0, 0.35, 45.0, 4.0, 0.65, 220.0)
	trajSet(link07, MSINE, 10.0,  0.0, 4.0, 0.35, 60.0, 4.0, 0.65, 220.0)
}
wait(10.0)
wait(0.30)

# ------------------------------------------------------------
# Phase 4: Quasi-static drift test (very slow)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, SINE, 12.0,  0.0, 2.0, 0.05)
	trajSet(link02, SINE, 12.0,  0.0, 2.5, 0.05)
	trajSet(link03, SINE, 12.0,  0.0, 2.5, 0.05)
	trajSet(link04, SINE, 12.0, -90.0, 2.0, 0.06)
	trajSet(link05, SINE, 12.0,  0.0, 2.0, 0.06)
	trajSet(link06, SINE, 12.0, 90.0, 2.5, 0.07)
	trajSet(link07, SINE, 12.0,  0.0, 2.5, 0.07)
}
wait(12.0)
wait(0.30)

# ------------------------------------------------------------
# Phase 5: High-frequency wrist stress (short, harsh)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link05, SINE, 6.0,  0.0, 3.0, 1.05)
	trajSet(link06, SINE, 6.0, 90.0, 4.0, 1.15)
	trajSet(link07, SINE, 6.0,  0.0, 3.0, 1.20)
}
wait(6.0)
wait(0.30)

# ------------------------------------------------------------
# Phase 6: Fatigue-style medium-frequency hold
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link04, SINE, 10.0, -90.0, 3.0, 0.40)
	trajSet(link05, SINE, 10.0,   0.0, 3.0, 0.45)
	trajSet(link06, SINE, 10.0,  90.0, 4.0, 0.50)
	trajSet(link07, SINE, 10.0,   0.0, 3.0, 0.50)
}
wait(10.0)
wait(0.30)

# ------------------------------------------------------------
# Phase 7: Final return to centre
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  60.0, 120.0)
	trajSet(link02, TRAP,   0.0,  50.0, 100.0)
	trajSet(link03, TRAP,   0.0,  50.0, 100.0)
	trajSet(link04, TRAP, -90.0,  60.0, 120.0)
	trajSet(link05, TRAP,   0.0,  70.0, 140.0)
	trajSet(link06, TRAP,  90.0,  70.0, 140.0)
	trajSet(link07, TRAP,   0.0,  70.0, 140.0)
}
wait(2.6)

trajClear()
wait(0.25)
stop()