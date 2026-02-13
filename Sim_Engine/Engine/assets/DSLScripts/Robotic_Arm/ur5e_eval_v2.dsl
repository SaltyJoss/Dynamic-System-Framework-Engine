# ============================================================
# INTEGRATOR NUMERICAL INTEGRATOR EVALUATION + STRESS SCRIPT (UR5e)
#
# Joint limits (deg):
# J1: [-360, +360]
# J2: [-180,    0]
# J3: [-180, +180]
# J4: [-360, +360]
# J5: [-360, +360]
# J6: [-360, +360]
#
# Joint Omega limits (deg/s):
# J1-J6: [0, +180]
#
# Created by: SaltyJoss
# Double Checked using GitHub Copilot
# ============================================================

load(robot, UR5e)
set(integrator, rk4)
wait(2.0)

trajClear()
wait(0.25)

start()
wait(0.35)

# ============================================================
# PHASE 0 — LARGE INITIAL OFFSET (BREAK SYMMETRY)
# ============================================================
parallel(0.0) {
    trajSet(link01, TRAP,   40.0, 120.0, 180.0)
    trajSet(link02, TRAP,  -60.0, 120.0, 180.0)
    trajSet(link03, TRAP, -140.0, 120.0, 180.0)
    trajSet(link04, TRAP,   60.0, 140.0, 180.0)
    trajSet(link05, TRAP,  -50.0, 140.0, 180.0)
    trajSet(link06, TRAP,   70.0, 160.0, 180.0)
}
wait(3.0)
wait(0.30)

# ============================================================
# PHASE 1 — LARGE-AMPLITUDE LOW-FREQUENCY FULL-CHAIN MOTION
# ============================================================
parallel(0.0) {
    trajSet(link01, SINE, 18.0,   0.0,  35.0, 0.18)
    trajSet(link02, SINE, 18.0, -80.0,  40.0, 0.17)
    trajSet(link03, SINE, 18.0,-140.0,  45.0, 0.16)
    trajSet(link04, SINE, 18.0,   0.0,  50.0, 0.19)
    trajSet(link05, SINE, 18.0,   0.0,  45.0, 0.19)
    trajSet(link06, SINE, 18.0,   0.0,  60.0, 0.20)
}
wait(18.0)
wait(0.25)

# ============================================================
# PHASE 2 — BROADBAND MSINE (FULL-CHAIN SPECTRAL STRESS)
# ============================================================
parallel(0.0) {
    trajSet(link01, MSINE, 25.0,   0.0, 30.0, 0.22,  20.0, 6.0, 0.45, 140.0, 3.0, 0.85, 300.0)
    trajSet(link02, MSINE, 25.0, -80.0, 35.0, 0.21,  15.0, 6.5, 0.42, 160.0, 3.5, 0.82, 320.0)
    trajSet(link03, MSINE, 25.0,-150.0, 45.0, 0.20,  12.0, 7.0, 0.40, 180.0, 4.0, 0.80, 340.0)
    trajSet(link04, MSINE, 25.0,   0.0, 40.0, 0.23,   0.0, 6.0, 0.55, 150.0, 3.0, 0.90, 300.0)
    trajSet(link05, MSINE, 25.0,   0.0, 40.0, 0.23,  30.0, 6.0, 0.55, 150.0, 3.0, 0.90, 300.0)
    trajSet(link06, MSINE, 25.0,   0.0, 55.0, 0.25,  45.0, 7.5, 0.50, 200.0, 4.0, 0.95, 360.0)
}
wait(25.0)
wait(0.30)

# ============================================================
# PHASE 3 — MID-FREQUENCY LARGE SWEEP (ENERGY DRIFT TEST)
# ============================================================
parallel(0.0) {
    trajSet(link01, SINE, 30.0,   0.0,  25.0, 0.55)
    trajSet(link02, SINE, 30.0, -90.0,  30.0, 0.52)
    trajSet(link03, SINE, 30.0,-135.0,  35.0, 0.50)
    trajSet(link04, SINE, 30.0,   0.0,  40.0, 0.60)
    trajSet(link05, SINE, 30.0,   0.0,  40.0, 0.60)
    trajSet(link06, SINE, 30.0,   0.0,  50.0, 0.65)
}
wait(30.0)
wait(0.25)

# ============================================================
# PHASE 4 — HIGH-FREQUENCY WRIST + BASE COUPLING
# ============================================================
parallel(0.0) {
    trajSet(link01, SINE, 14.0,   0.0,  18.0, 1.00)
    trajSet(link04, SINE, 14.0,   0.0,  25.0, 1.20)
    trajSet(link05, SINE, 14.0,   0.0,  25.0, 1.25)
    trajSet(link06, SINE, 14.0,   0.0,  35.0, 1.10)
}
wait(14.0)
wait(0.25)

# ============================================================
# PHASE 5 — LARGE REVERSALS (NUMERICAL ROBUSTNESS)
# ============================================================
parallel(0.0) {
    trajSet(link01, TRAP,   90.0, 160.0, 180.0)
    trajSet(link02, TRAP,  -30.0, 160.0, 180.0)
    trajSet(link03, TRAP, -170.0, 160.0, 180.0)
    trajSet(link04, TRAP,  -80.0, 160.0, 180.0)
    trajSet(link05, TRAP,   70.0, 160.0, 180.0)
    trajSet(link06, TRAP,  -90.0, 160.0, 180.0)
}
wait(3.2)
wait(0.30)

# ============================================================
# PHASE 6 — LONG FATIGUE OSCILLATION (INTEGRATOR BLEED)
# ============================================================
parallel(0.0) {
    trajSet(link01, SINE, 35.0,   0.0,  15.0, 0.35)
    trajSet(link02, SINE, 35.0, -85.0,  18.0, 0.33)
    trajSet(link03, SINE, 35.0,-145.0,  22.0, 0.32)
    trajSet(link04, SINE, 35.0,   0.0,  25.0, 0.38)
    trajSet(link05, SINE, 35.0,   0.0,  25.0, 0.38)
    trajSet(link06, SINE, 35.0,   0.0,  30.0, 0.40)
}
wait(35.0)
wait(0.25)

# ============================================================
# PHASE 7 — HARD RETURN (CLEANUP / CONSISTENCY CHECK)
# ============================================================
parallel(0.0) {
    trajSet(link01, TRAP,    0.0, 120.0, 180.0)
    trajSet(link02, TRAP,  -90.0, 120.0, 180.0)
    trajSet(link03, TRAP, -120.0, 120.0, 180.0)
    trajSet(link04, TRAP,    0.0, 140.0, 180.0)
    trajSet(link05, TRAP,    0.0, 140.0, 180.0)
    trajSet(link06, TRAP,    0.0, 160.0, 180.0)
}
wait(3.0)

trajClear()
wait(0.25)
stop()

