# ============================================================
# INTEGRATOR EVALUATION + SHOWCASE SCRIPT (iiwa14)
#
# Joint limits (deg, conservative):
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
# Assisted by: GitHub Copilot
# ============================================================

load(robot, iiwa14)
set(integrator, rk4)
wait(2)

trajClear()
wait(0.25)

start()
wait(0.35)

# ------------------------------------------------------------
# Phase 0: Move to benchmark centre pose
# Centre (deg): [0, 30, 0, -60, 0, 60, 0]
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0,  80.0, 160.0)
    trajSet(link02, TRAP,  30.0,  70.0, 140.0)
    trajSet(link03, TRAP,   0.0,  70.0, 140.0)
    trajSet(link04, TRAP, -60.0,  70.0, 140.0)
    trajSet(link05, TRAP,   0.0,  90.0, 160.0)
    trajSet(link06, TRAP,  60.0,  90.0, 160.0)
    trajSet(link07, TRAP,   0.0, 120.0, 180.0)
}
wait(2.6)
wait(0.30)

# ------------------------------------------------------------
# Phase 1: Low-frequency breathing motion
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, SINE, 6.0,   0.0,  6.0, 0.18)
    trajSet(link02, SINE, 6.0,  30.0,  8.0, 0.16)
    trajSet(link03, SINE, 6.0,   0.0,  8.0, 0.16)
    trajSet(link04, SINE, 6.0, -60.0, 10.0, 0.15)
    trajSet(link05, SINE, 6.0,   0.0,  8.0, 0.18)
    trajSet(link06, SINE, 6.0,  60.0, 10.0, 0.18)
    trajSet(link07, SINE, 6.0,   0.0, 12.0, 0.20)
}
wait(6.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 2: Re-centre
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0,  90.0, 180.0)
    trajSet(link02, TRAP,  30.0,  80.0, 160.0)
    trajSet(link03, TRAP,   0.0,  80.0, 160.0)
    trajSet(link04, TRAP, -60.0,  80.0, 160.0)
    trajSet(link05, TRAP,   0.0, 100.0, 180.0)
    trajSet(link06, TRAP,  60.0, 100.0, 180.0)
    trajSet(link07, TRAP,   0.0, 140.0, 180.0)
}
wait(1.8)
wait(0.20)

# ------------------------------------------------------------
# Phase 3: Broadband MSINE inspection sweep
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, MSINE, 10.0,   0.0, 4.0, 0.20,  20.0, 3.0, 0.45, 140.0, 2.0, 0.85, 260.0)
    trajSet(link02, MSINE, 10.0,  30.0, 5.0, 0.18,  10.0, 4.0, 0.40, 150.0, 2.5, 0.75, 270.0)
    trajSet(link03, MSINE, 10.0,   0.0, 5.0, 0.18,  10.0, 4.0, 0.40, 150.0, 2.5, 0.75, 270.0)
    trajSet(link04, MSINE, 10.0, -60.0, 6.0, 0.16,   0.0, 3.0, 0.55, 110.0, 2.0, 0.95, 230.0)
    trajSet(link05, MSINE, 10.0,   0.0, 5.0, 0.22,  30.0, 3.0, 0.55, 110.0, 2.0, 0.95, 230.0)
    trajSet(link06, MSINE, 10.0,  60.0, 6.0, 0.22,  45.0, 5.0, 0.50, 160.0, 3.0, 0.90, 300.0)
    trajSet(link07, MSINE, 10.0,   0.0, 8.0, 0.24,  60.0, 5.0, 0.55, 170.0, 3.0, 0.85, 310.0)
}
wait(10.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 4: Re-centre
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0, 100.0, 180.0)
    trajSet(link02, TRAP,  30.0,  90.0, 180.0)
    trajSet(link03, TRAP,   0.0,  90.0, 180.0)
    trajSet(link04, TRAP, -60.0,  90.0, 180.0)
    trajSet(link05, TRAP,   0.0, 120.0, 180.0)
    trajSet(link06, TRAP,  60.0, 120.0, 180.0)
    trajSet(link07, TRAP,   0.0, 160.0, 180.0)
}
wait(2.0)
wait(0.20)

# ------------------------------------------------------------
# Phase 5: Medium-frequency band
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, SINE, 7.0,   0.0, 5.0, 0.55)
    trajSet(link02, SINE, 7.0,  30.0, 6.0, 0.50)
    trajSet(link03, SINE, 7.0,   0.0, 6.0, 0.50)
    trajSet(link04, SINE, 7.0, -60.0, 7.0, 0.45)
    trajSet(link05, SINE, 7.0,   0.0, 6.0, 0.60)
    trajSet(link06, SINE, 7.0,  60.0, 7.0, 0.60)
    trajSet(link07, SINE, 7.0,   0.0, 9.0, 0.65)
}
wait(7.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 6: Re-centre again
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0, 110.0, 180.0)
    trajSet(link02, TRAP,  30.0, 100.0, 180.0)
    trajSet(link03, TRAP,   0.0, 100.0, 180.0)
    trajSet(link04, TRAP, -60.0, 100.0, 180.0)
    trajSet(link05, TRAP,   0.0, 140.0, 180.0)
    trajSet(link06, TRAP,  60.0, 140.0, 180.0)
    trajSet(link07, TRAP,   0.0, 180.0, 180.0)
}
wait(2.2)
wait(0.20)

# ------------------------------------------------------------
# Phase 7: High-frequency wrist / redundancy stress
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link05, SINE, 5.0, 0.0, 4.0, 1.10)
    trajSet(link06, SINE, 5.0, 60.0, 4.0, 1.15)
    trajSet(link07, SINE, 5.0, 0.0, 6.0, 1.25)
}
wait(5.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 8: Showcase choreography
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   25.0,  80.0, 150.0)
    trajSet(link02, TRAP,   60.0,  60.0, 120.0)
    trajSet(link03, TRAP,  -40.0,  60.0, 120.0)
    trajSet(link04, TRAP,  -80.0,  70.0, 140.0)
    trajSet(link05, TRAP,   20.0,  90.0, 160.0)
    trajSet(link06, TRAP,   90.0,  90.0, 160.0)
    trajSet(link07, TRAP,   30.0, 120.0, 180.0)
}
wait(2.8)
wait(0.35)

parallel(0.0) {
    trajSet(link01, TRAP,  -25.0,  80.0, 150.0)
    trajSet(link02, TRAP,    0.0,  60.0, 120.0)
    trajSet(link03, TRAP,   40.0,  60.0, 120.0)
    trajSet(link04, TRAP,  -40.0,  70.0, 140.0)
    trajSet(link05, TRAP,  -20.0,  90.0, 160.0)
    trajSet(link06, TRAP,   30.0,  90.0, 160.0)
    trajSet(link07, TRAP,  -30.0, 120.0, 180.0)
}
wait(2.8)
wait(0.35)

# ------------------------------------------------------------
# Phase 9: Return to benchmark centre
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0,  90.0, 160.0)
    trajSet(link02, TRAP,  30.0,  70.0, 140.0)
    trajSet(link03, TRAP,   0.0,  70.0, 140.0)
    trajSet(link04, TRAP, -60.0,  70.0, 140.0)
    trajSet(link05, TRAP,   0.0, 100.0, 180.0)
    trajSet(link06, TRAP,  60.0, 100.0, 180.0)
    trajSet(link07, TRAP,   0.0, 140.0, 180.0)
}
wait(2.4)
wait(0.25)

# ------------------------------------------------------------
# Phase 10: Return HOME
# Home (deg): [0, 0, 0, 0, 0, 0, 0]
# ------------------------------------------------------------
parallel(0.0) {
    trajSet(link01, TRAP,   0.0,  90.0, 160.0)
    trajSet(link02, TRAP,   0.0,  70.0, 140.0)
    trajSet(link03, TRAP,   0.0,  70.0, 140.0)
    trajSet(link04, TRAP,   0.0,  70.0, 140.0)
    trajSet(link05, TRAP,   0.0, 100.0, 180.0)
    trajSet(link06, TRAP,   0.0, 100.0, 180.0)
    trajSet(link07, TRAP,   0.0, 140.0, 180.0)
}
wait(2.6)

trajClear()
wait(0.25)
stop()
