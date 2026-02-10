# ============================================================
# INTEGRATOR EVALUATION + SHOWCASE SCRIPT (Z1)
# - Designed to avoid hitting limits (no clamp engagement expected)
#
# Joint limits (deg):
# J1: [-150, +150]
# J2: [   0, +170]
# J3: [-165,    0]
# J4: [ -87,  +87]
# J5: [ -77,  +77]
# J6: [-160, +160]
#
# Joint Omega limits (deg/s):
# J1-J6: [0, +180]
# ============================================================

load(robot, Z1)
set(integrator, rk4)
wait(2)

trajClear()
wait(0.25)

start()
wait(0.35)

# ------------------------------------------------------------
# Phase 0: Move to benchmark center pose (smooth, inside limits)
# Centre(deg): [0, 85, -85, 0, 0, 0]
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  80.0, 160.0)
	trajSet(link02, TRAP,  85.0,  60.0, 140.0)
	trajSet(link03, TRAP, -85.0,  60.0, 140.0)
	trajSet(link04, TRAP,   0.0,  90.0, 180.0)
	trajSet(link05, TRAP,   0.0,  90.0, 180.0)
	trajSet(link06, TRAP,   0.0, 120.0, 180.0)
}
wait(2.6)
wait(0.30)

# ------------------------------------------------------------
# Phase 1: Low-frequency “breathing” motion (should be easy)
# SINE format: trajSet(link, SINE, dur, centerDeg, ampDeg, freqHz [,phaseDeg])
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, SINE, 6.0,   0.0,  6.0, 0.18)
	trajSet(link02, SINE, 6.0,  85.0, 10.0, 0.16)
	trajSet(link03, SINE, 6.0, -85.0, 10.0, 0.16)
	trajSet(link04, SINE, 6.0,   0.0,  8.0, 0.20)
	trajSet(link05, SINE, 6.0,   0.0,  8.0, 0.20)
	trajSet(link06, SINE, 6.0,   0.0, 12.0, 0.22)
}
wait(6.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 2: Re-centre (removes accumulated phase error / drift)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  90.0, 180.0)
	trajSet(link02, TRAP,  85.0,  70.0, 160.0)
	trajSet(link03, TRAP, -85.0,  70.0, 160.0)
	trajSet(link04, TRAP,   0.0, 100.0, 180.0)
	trajSet(link05, TRAP,   0.0, 100.0, 180.0)
	trajSet(link06, TRAP,   0.0, 130.0, 180.0)
}
wait(1.8)
wait(0.20)

# ------------------------------------------------------------
# Phase 3: Broadband MSINE “inspection sweep” (moderate stress)
# MSINE format: trajSet(link, MSINE, dur, centerDeg, amp1Deg, f1Hz, ph1Deg, amp2Deg, f2Hz, ph2Deg, amp3Deg, f3Hz, ph3Deg)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, MSINE, 10.0,   0.0, 4.0, 0.20, 20.0, 3.0, 0.45, 140.0, 2.0, 0.85, 260.0)
	trajSet(link02, MSINE, 10.0,  85.0, 6.0, 0.18, 10.0, 4.0, 0.40, 150.0, 2.5, 0.75, 270.0)
	trajSet(link03, MSINE, 10.0, -85.0, 6.0, 0.18, 10.0, 4.0, 0.40, 150.0, 2.5, 0.75, 270.0)
	trajSet(link04, MSINE, 10.0,   0.0, 5.0, 0.22,  0.0, 3.0, 0.55, 110.0, 2.0, 0.95, 230.0)
	trajSet(link05, MSINE, 10.0,   0.0, 5.0, 0.22, 30.0, 3.0, 0.55, 110.0, 2.0, 0.95, 230.0)
	trajSet(link06, MSINE, 10.0,   0.0, 8.0, 0.24, 45.0, 5.0, 0.50, 160.0, 3.0, 0.90, 300.0)
}
wait(10.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 4: Re-centre (baseline before higher-frequency band)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0, 100.0, 220.0)
	trajSet(link02, TRAP,  85.0,  80.0, 180.0)
	trajSet(link03, TRAP, -85.0,  80.0, 180.0)
	trajSet(link04, TRAP,   0.0, 120.0, 180.0)
	trajSet(link05, TRAP,   0.0, 120.0, 180.0)
	trajSet(link06, TRAP,   0.0, 150.0, 180.0)
}
wait(2.0)
wait(0.20)

# ------------------------------------------------------------
# Phase 5: Medium-frequency band (phase/energy error becomes obvious)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, SINE, 7.0,   0.0, 5.0, 0.55)
	trajSet(link02, SINE, 7.0,  85.0, 7.0, 0.50)
	trajSet(link03, SINE, 7.0, -85.0, 7.0, 0.50)
	trajSet(link04, SINE, 7.0,   0.0, 6.0, 0.65)
	trajSet(link05, SINE, 7.0,   0.0, 6.0, 0.65)
	trajSet(link06, SINE, 7.0,   0.0, 9.0, 0.70)
}
wait(7.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 6: Re-centre again
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0, 110.0, 180.0)
	trajSet(link02, TRAP,  85.0,  90.0, 180.0)
	trajSet(link03, TRAP, -85.0,  90.0, 180.0)
	trajSet(link04, TRAP,   0.0, 140.0, 180.0)
	trajSet(link05, TRAP,   0.0, 140.0, 180.0)
	trajSet(link06, TRAP,   0.0, 170.0, 180.0)
}
wait(2.2)
wait(0.20)

# ------------------------------------------------------------
# Phase 7: High-frequency wrist scan (short burst = harshest test)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link04, SINE, 5.0, 0.0, 4.0, 1.20)
	trajSet(link05, SINE, 5.0, 0.0, 4.0, 1.25)
	trajSet(link06, SINE, 5.0, 0.0, 6.0, 1.10)
}
wait(5.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 8: Showcase choreography (smooth asymmetric poses)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   30.0,  70.0, 150.0)
	trajSet(link02, TRAP,  100.0,  55.0, 120.0)
	trajSet(link03, TRAP, -110.0,  55.0, 120.0)
	trajSet(link04, TRAP,   15.0,  80.0, 160.0)
	trajSet(link05, TRAP,  -10.0,  80.0, 160.0)
	trajSet(link06, TRAP,   20.0, 110.0, 180.0)
}
wait(2.8)
wait(0.35)

parallel(0.0) {
	trajSet(link01, TRAP, -30.0,  70.0, 150.0)
	trajSet(link02, TRAP,  70.0,  55.0, 120.0)
	trajSet(link03, TRAP, -70.0,  55.0, 120.0)
	trajSet(link04, TRAP, -15.0,  80.0, 160.0)
	trajSet(link05, TRAP,  10.0,  80.0, 160.0)
	trajSet(link06, TRAP, -20.0, 110.0, 180.0)
}
wait(2.8)
wait(0.35)

# ------------------------------------------------------------
# Phase 9: Return to benchmark centre
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  80.0, 160.0)
	trajSet(link02, TRAP,  85.0,  60.0, 140.0)
	trajSet(link03, TRAP, -85.0,  60.0, 140.0)
	trajSet(link04, TRAP,   0.0,  90.0, 180.0)
	trajSet(link05, TRAP,   0.0,  90.0, 180.0)
	trajSet(link06, TRAP,   0.0, 120.0, 180.0)
}
wait(2.4)
wait(0.25)

# ------------------------------------------------------------
# Phase 10: Return HOME (original home)
# Home (deg): [0, 45, -60, 0, 0, 0]
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   0.0,  80.0, 160.0)
	trajSet(link02, TRAP,  45.0,  60.0, 140.0)
	trajSet(link03, TRAP, -60.0,  60.0, 140.0)
	trajSet(link04, TRAP,   0.0,  90.0, 180.0)
	trajSet(link05, TRAP,   0.0,  90.0, 180.0)
	trajSet(link06, TRAP,   0.0, 120.0, 180.0)
}
wait(2.6)

trajClear()
wait(0.25)
stop()