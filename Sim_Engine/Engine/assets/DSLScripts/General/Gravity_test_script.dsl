# ============================================================
# EXTENDED GRAVITY STRESS TEST (Z1) — long run
# - Use to reveal drift, energy non-conservation, phase lag, noise amplification
# - Only uses: set, load, start, stop, wait, trajClear, parallel, trajSet
# - Respect joint limits (J1..J6) from spec
#
# Created by: SaltyJoss
# ============================================================

set(integrator, rk4)    # change integrator to compare runs
load(robot, Z1)
wait(2.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# ------------------------------------------------------------
# Phase 0: Warmup -> centred benchmark pose (longer settle)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,    5.0,  70.0, 140.0)
	trajSet(link02, TRAP,   95.0,  60.0, 140.0)
	trajSet(link03, TRAP,  -75.0,  60.0, 140.0)
	trajSet(link04, TRAP,    5.0,  90.0, 180.0)
	trajSet(link05, TRAP,  -10.0,  90.0, 180.0)
	trajSet(link06, TRAP,   10.0, 120.0, 180.0)
}
wait(4.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 1: Gravity-max pose A (long hold + micro-sine on wrist)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,   25.0,  60.0, 120.0)
	trajSet(link02, TRAP,  130.0,  50.0, 120.0)
	trajSet(link03, TRAP,  -20.0,  50.0, 120.0)
	trajSet(link04, TRAP,   40.0,  60.0, 120.0)
	trajSet(link05, TRAP,  -30.0,  60.0, 120.0)
	trajSet(link06, TRAP,   20.0, 100.0, 160.0)
}
wait(6.0)

# micro-wrist excitation during hang
parallel(0.0) {
	trajSet(link06, SINE, 20.0,  20.0, 0.8, 1.05)
}
wait(20.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 2: Slow amplitude ramp (series of increasing SINE segments)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link02, SINE, 10.0, 125.0,  4.0, 0.12)
	trajSet(link03, SINE, 10.0, -25.0,  4.0, 0.12)
}
wait(10.0)
wait(0.25)

parallel(0.0) {
	trajSet(link02, SINE, 12.0, 125.0,  8.0, 0.16)
	trajSet(link03, SINE, 12.0, -25.0,  8.0, 0.16)
}
wait(12.0)
wait(0.25)

parallel(0.0) {
	trajSet(link02, SINE, 14.0, 125.0, 12.0, 0.20)
	trajSet(link03, SINE, 14.0, -25.0, 12.0, 0.20)
}
wait(14.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 3: Cross-joint MSINE cascade (long)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link02, MSINE, 24.0, 120.0, 6.0, 0.18,   0.0, 5.0, 0.40, 120.0, 3.0, 0.85, 240.0)
	trajSet(link03, MSINE, 24.0, -30.0, 6.0, 0.18, 180.0, 5.0, 0.40, 300.0, 3.0, 0.85,  60.0)
	trajSet(link04, MSINE, 24.0,  35.0, 4.0, 0.22,  90.0, 3.5, 0.50, 200.0, 2.5, 0.95, 330.0)
	trajSet(link05, MSINE, 24.0, -25.0, 4.0, 0.22, 150.0, 3.5, 0.50, 260.0, 2.5, 0.95,  10.0)
}
wait(24.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 4: Rapid asymmetric TRAP swaps (energy injection)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,  -30.0,  80.0, 160.0)
	trajSet(link02, TRAP,  150.0,  40.0, 100.0)
	trajSet(link03, TRAP,  -10.0,  40.0, 100.0)
	trajSet(link04, TRAP,   60.0,  60.0, 120.0)
	trajSet(link05, TRAP,  -60.0,  60.0, 120.0)
	trajSet(link06, TRAP,   40.0, 100.0, 160.0)
}
wait(3.2)
wait(0.3)

parallel(0.0) {
	trajSet(link01, TRAP,   45.0,  70.0, 140.0)
	trajSet(link02, TRAP,  100.0,  50.0, 120.0)
	trajSet(link03, TRAP,  -60.0,  50.0, 120.0)
	trajSet(link04, TRAP,   20.0,  80.0, 140.0)
	trajSet(link05, TRAP,  -15.0,  80.0, 140.0)
	trajSet(link06, TRAP,   10.0, 120.0, 180.0)
}
wait(3.2)
wait(0.3)

# ------------------------------------------------------------
# Phase 5: Long asymmetric hang (drift detector) — extended
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,    0.0,  70.0, 140.0)
	trajSet(link02, TRAP,  140.0,  30.0,  80.0)
	trajSet(link03, TRAP,  -10.0,  30.0,  80.0)
	trajSet(link04, TRAP,   50.0,  50.0, 100.0)
	trajSet(link05, TRAP,  -50.0,  50.0, 100.0)
	trajSet(link06, TRAP,   60.0,  90.0, 140.0)
}
wait(30.0)
wait(0.6)

# during long hang, apply tiny opposing micro-sines to show accumulation
parallel(0.0) {
	trajSet(link02, SINE, 30.0, 140.0, 0.6, 0.45)
	trajSet(link03, SINE, 30.0, -10.0, 0.6, 0.45)
	trajSet(link06, SINE, 30.0,  60.0, 0.9, 0.95)
}
wait(30.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 6: High-frequency wrist/forearm stress bursts (short)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link04, SINE, 8.0,  55.0, 3.5, 1.60)
	trajSet(link05, SINE, 8.0, -55.0, 3.5, 1.65)
	trajSet(link06, SINE, 8.0,  65.0, 4.5, 1.50)
}
wait(8.0)
wait(0.3)

# repeat Phase 3 but reversed phase offsets (exercise hysteresis)
parallel(0.0) {
	trajSet(link02, MSINE, 20.0, 110.0, 5.5, 0.16, 180.0, 4.5, 0.38, 300.0, 3.0, 0.75, 120.0)
	trajSet(link03, MSINE, 20.0, -40.0, 5.5, 0.16,   0.0, 4.5, 0.38, 120.0, 3.0, 0.75, 300.0)
	trajSet(link04, MSINE, 20.0,  30.0, 4.5, 0.20,  45.0, 3.5, 0.48, 180.0, 2.5, 0.92, 270.0)
}
wait(20.0)
wait(0.5)

# ------------------------------------------------------------
# Phase 7: Gentle recovery sweep + long settle
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,    0.0,  80.0, 160.0)
	trajSet(link02, TRAP,   95.0,  60.0, 140.0)
	trajSet(link03, TRAP,  -85.0,  60.0, 140.0)
	trajSet(link04, TRAP,   10.0,  90.0, 160.0)
	trajSet(link05, TRAP,  -10.0,  90.0, 160.0)
	trajSet(link06, TRAP,   15.0, 120.0, 180.0)
}
wait(6.0)
wait(0.5)

# long settle to reveal residual drift
parallel(0.0) {
	trajSet(link02, SINE, 40.0,  95.0, 0.8, 0.08)
	trajSet(link03, SINE, 40.0, -85.0, 0.8, 0.08)
}
wait(40.0)
wait(0.6)

# ------------------------------------------------------------
# Phase 8: Final showcase + return to HOME (smooth)
# ------------------------------------------------------------
parallel(0.0) {
	trajSet(link01, TRAP,    10.0,  90.0, 180.0)
	trajSet(link02, TRAP,    45.0,  60.0, 140.0)
	trajSet(link03, TRAP,   -60.0,  60.0, 140.0)
	trajSet(link04, TRAP,     0.0,  90.0, 180.0)
	trajSet(link05, TRAP,     0.0,  90.0, 180.0)
	trajSet(link06, TRAP,     0.0, 120.0, 180.0)
}
wait(6.0)

trajClear()
wait(0.3)
stop()
