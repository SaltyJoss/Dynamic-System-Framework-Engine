# ------------------------------------------------------------
# Z1 BIG demo (degrees input for trajSet)
# - Phase A: TRAP pose to "wake up" (all joints)
# - Phase B: SINE "breathing" (all joints, bounded)
# - Phase C: MSINE richer wobble (all joints, bounded)
# - Phase D: TRAP to a different pose
# - Phase E: SINE sweep (slower, larger on base/shoulder)
# ------------------------------------------------------------

load(robot, Z1)
set(integrator, rk4)
wait(1.0)

trajClear()
start()
wait(1.0)

# -------------------------
# Phase A: TRAP to Pose A (3.5s)
# -------------------------
parallel(3.5) {
  # TRAP: q1_deg, vmax_deg_s, amax_deg_s2
  trajSet(link01, TRAP,  -90.0, 140.0, 280.0)   # base
  trajSet(link02, TRAP,   80.0, 120.0, 240.0)   # shoulder
  trajSet(link03, TRAP,  -70.0, 120.0, 240.0)   # elbow
  trajSet(link04, TRAP,  -45.0, 140.0, 280.0)   # wrist pitch
  trajSet(link05, TRAP,  -35.0, 140.0, 280.0)   # wrist yaw
  trajSet(link06, TRAP,   60.0, 160.0, 320.0)   # wrist roll
}

wait(0.5)

# -------------------------
# Phase B: SINE "breathing" (10s)
# -------------------------
parallel(10.0) {
  # SINE: amp_deg, freq_hz, dur_s, phase_deg
  trajSet(link01, SINE,  35.0, 0.18, 10.0,   0.0)
  trajSet(link02, SINE,  25.0, 0.18, 10.0,  40.0)
  trajSet(link03, SINE,  25.0, 0.18, 10.0,  80.0)
  trajSet(link04, SINE, -20.0, 0.22, 10.0, 120.0)
  trajSet(link05, SINE,  20.0, 0.22, 10.0, 160.0)
  trajSet(link06, SINE,  25.0, 0.22, 10.0, 200.0)
}

wait(0.5)

# -------------------------
# Phase C: MSINE "richer wobble" (14s)
# -------------------------
parallel(14.0) {
  # MSINE: dur_s, (amp_deg, f_hz, phase_deg) xN
  trajSet(link01, MSINE, 14.0, 35.0, 0.10, 0.0, 16.0, 0.35, 110.0, 7.0, 0.80, 200.0)
  trajSet(link02, MSINE, 14.0, 12.0, 0.12, 30.0, 6.0, 0.40, 140.0, 3.0, 0.90, 220.0)
  trajSet(link03, MSINE, 14.0, 12.0, 0.12, 60.0, 6.0, 0.40, 160.0, 3.0, 0.90, 240.0)
  trajSet(link04, MSINE, 14.0, 16.0, 0.16, 0.0,  8.0, 0.55, 100.0, 3.5, 1.10, 190.0)
  trajSet(link05, MSINE, 14.0, 14.0, 0.16, 30.0, 7.0, 0.55, 130.0, 3.5, 1.10, 210.0)
  trajSet(link06, MSINE, 14.0, 20.0, 0.20, 0.0, 10.0, 0.70,  95.0, 5.0, 1.40, 185.0)
}

wait(0.5)

# -------------------------
# Phase D: TRAP to Pose B (4.0s)
# -------------------------
parallel(4.0) {
  trajSet(link01, TRAP,  110.0, 160.0, 320.0)
  trajSet(link02, TRAP,   35.0, 140.0, 280.0)
  trajSet(link03, TRAP, -120.0, 140.0, 280.0)
  trajSet(link04, TRAP,  -30.0, 160.0, 320.0)
  trajSet(link05, TRAP,   50.0, 160.0, 320.0)
  trajSet(link06, TRAP,  -80.0, 180.0, 360.0)
}

wait(0.5)

# -------------------------
# Phase E: SINE "show-off sweep" (12s)
# -------------------------
parallel(12.0) {
  trajSet(link01, SINE,   55.0, 0.12, 12.0,   0.0)
  trajSet(link02, SINE,   18.0, 0.16, 12.0,  45.0)
  trajSet(link03, SINE,   18.0, 0.16, 12.0,  90.0)
  trajSet(link04, SINE,  -22.0, 0.20, 12.0, 135.0)
  trajSet(link05, SINE,   22.0, 0.20, 12.0, 180.0)
  trajSet(link06, SINE,   30.0, 0.24, 12.0, 225.0)
}

trajClear()
wait(0.5)
stop()