# -----------------------------------------------
# Z1 trajSet validation (DEGREES input)
# - TRAP for all 6 joints (visible, bounded)
# - then SINE for all 6 (bounded)
# - then MSINE for all 6 (bounded)
# -----------------------------------------------

load(robot, Z1)
set(integrator, rk4)
wait(1.0)

trajClear()
start()
wait(1.0)

# ----------------
# 1) TRAP (2.5s)
# ----------------
parallel(2.5) {
  # TRAP: trajSet(link, TRAP, q1_deg, vmax_deg_s, amax_deg_s2)

  # joint01 limits ~ [-150, +150] deg
  trajSet(link01, TRAP,  -60.0, 120.0, 240.0)

  # joint02 limits ~ [0, +170] deg
  trajSet(link02, TRAP,   60.0, 120.0, 240.0)

  # joint03 limits ~ [-165, 0] deg
  trajSet(link03, TRAP,  -60.0, 120.0, 240.0)

  # joint04 limits ~ [-87, +87] deg
  trajSet(link04, TRAP,   30.0, 120.0, 240.0)

  # joint05 limits ~ [-77, +77] deg
  trajSet(link05, TRAP,  -25.0, 120.0, 240.0)

  # joint06 limits ~ [-160, +160] deg
  trajSet(link06, TRAP,   40.0, 120.0, 240.0)
}

wait(0.5)

# ----------------
# 2) SINE (6s)
# ----------------
parallel(6.0) {
  # SINE: trajSet(link, SINE, amp_deg, freq_hz, dur_s, phase_deg)
  trajSet(link01, SINE, 25.0, 0.35, 6.0,   0.0)
  trajSet(link02, SINE, 20.0, 0.35, 6.0,  30.0)
  trajSet(link03, SINE, 20.0, 0.35, 6.0,  60.0)
  trajSet(link04, SINE, 15.0, 0.35, 6.0,  90.0)
  trajSet(link05, SINE, 15.0, 0.35, 6.0, 120.0)
  trajSet(link06, SINE, 20.0, 0.35, 6.0, 150.0)
}

wait(0.5)

# ----------------
# 3) MSINE (8s)
# ----------------
parallel(8.0) {
  # MSINE: trajSet(link, MSINE, dur_s, amp_deg, f_hz, phase_deg, amp_deg, f_hz, phase_deg, ...)

  trajSet(link01, MSINE, 8.0, 12.0, 0.25, 0.0, 8.0, 0.60, 90.0)

  trajSet(link02, MSINE, 8.0, 10.0, 0.20, 0.0, 6.0, 0.55, 60.0)

  trajSet(link03, MSINE, 8.0, 10.0, 0.22, 0.0, 6.0, 0.50, 45.0)

  trajSet(link04, MSINE, 8.0, 8.0, 0.30, 0.0, 5.0, 0.70, 90.0)

  trajSet(link05, MSINE, 8.0, 8.0, 0.28, 0.0, 5.0, 0.65, 120.0)

  trajSet(link06, MSINE, 8.0, 10.0, 0.26, 0.0, 6.0, 0.62, 30.0)
}

trajClear()
wait(0.5)
stop()