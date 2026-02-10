# -----------------------------
# Pose definitions (degrees)
# -----------------------------
# Joint limits reminder (deg):
# J1: [-150, +150]
# J2: [   0, +170]
# J3: [-165,   0]
# J4: [ -87,  +87]
# J5: [ -77,  +77]
# J6: [-160, +160]

load(robot, Z1)
set(integrator, rk4)
wait(0.5)

trajClear()
start()
wait(0.5)

# -----------------------------
# Phase 0: Move to a neutral HOME pose (2.5s)
# -----------------------------
parallel(0.0) {
  trajSet(link01, TRAP,   0.0,  60.0, 120.0)
  trajSet(link02, TRAP,  45.0,  50.0, 100.0)
  trajSet(link03, TRAP, -60.0,  50.0, 100.0)
  trajSet(link04, TRAP,   0.0,  60.0, 120.0)
  trajSet(link05, TRAP,   0.0,  60.0, 120.0)
  trajSet(link06, TRAP,   0.0,  80.0, 160.0)
}
wait(2.5)
wait(0.4)

# -----------------------------
# Phase 1: Approach pose above "pick" location (3.0s)
# (Base turns, shoulder/elbow shape, wrist aligns)
# -----------------------------
parallel(0.0) {
  trajSet(link01, TRAP,  35.0,  55.0, 110.0)
  trajSet(link02, TRAP,  70.0,  45.0,  90.0)
  trajSet(link03, TRAP, -95.0,  45.0,  90.0)
  trajSet(link04, TRAP,  20.0,  60.0, 120.0)
  trajSet(link05, TRAP, -10.0,  60.0, 120.0)
  trajSet(link06, TRAP,  15.0,  80.0, 160.0)
}
wait(3.0)
wait(0.3)

# -----------------------------
# Phase 2: Lower to "pick" (fine motion, mostly shoulder/elbow) (1.8s)
# -----------------------------
parallel(0.0) {
  trajSet(link02, TRAP,  82.0,  25.0,  60.0)
  trajSet(link03, TRAP, -110.0, 25.0,  60.0)
  trajSet(link04, TRAP,  15.0,  30.0,  70.0)
}
wait(1.8)

# Dwell to simulate grasp/close gripper (you don't have gripper yet)
wait(0.6)

# -----------------------------
# Phase 3: Lift slightly (1.5s)
# -----------------------------
parallel(0.0) {
  trajSet(link02, TRAP,  72.0,  25.0,  60.0)
  trajSet(link03, TRAP, -98.0,  25.0,  60.0)
}
wait(1.5)
wait(0.3)

# -----------------------------
# Phase 4: Transfer to "place" approach (3.2s)
# (Base swing + elbow reshape, wrist re-orient)
# -----------------------------
parallel(0.0) {
  trajSet(link01, TRAP, -40.0,  55.0, 110.0)
  trajSet(link02, TRAP,  60.0,  40.0,  85.0)
  trajSet(link03, TRAP, -85.0,  40.0,  85.0)
  trajSet(link04, TRAP, -15.0,  55.0, 110.0)
  trajSet(link05, TRAP,  20.0,  55.0, 110.0)
  trajSet(link06, TRAP, -25.0,  80.0, 160.0)
}
wait(3.2)
wait(0.3)

# -----------------------------
# Phase 5: Lower to "place" (1.8s)
# -----------------------------
parallel(0.0) {
  trajSet(link02, TRAP,  75.0,  25.0,  60.0)
  trajSet(link03, TRAP, -105.0, 25.0,  60.0)
  trajSet(link04, TRAP, -20.0,  30.0,  70.0)
}
wait(1.8)

# Dwell to simulate release/open gripper
wait(0.6)

# -----------------------------
# Phase 6: Retract to a safe mid pose (2.2s)
# -----------------------------
parallel(0.0) {
  trajSet(link02, TRAP,  55.0,  40.0,  85.0)
  trajSet(link03, TRAP, -70.0,  40.0,  85.0)
  trajSet(link04, TRAP,   0.0,  55.0, 110.0)
  trajSet(link05, TRAP,   0.0,  55.0, 110.0)
}
wait(2.2)
wait(0.3)

# -----------------------------
# Phase 6.1: small, low-frequency inspection sweep
# -----------------------------
parallel(0.0) {
  trajSet(link01, MSINE, 8.0, 0.2, 3.0, 0.08, 10.0, 2.0, 0.16, 110.0, 1.0, 0.28, 210.0)
  trajSet(link02, MSINE, 8.0, 0.2, 2.5, 0.07, 40.0, 1.5, 0.14, 140.0, 1.0, 0.25, 240.0)
  trajSet(link03, MSINE, 8.0, 0.2, 2.5, 0.07, 80.0, 1.5, 0.14, 180.0, 1.0, 0.25, 280.0)
  trajSet(link04, MSINE, 8.0, 0.2, 4.0, 0.10,  0.0, 2.0, 0.20,  90.0, 1.0, 0.35, 180.0)
  trajSet(link05, MSINE, 8.0, 0.2, 4.0, 0.10, 30.0, 2.0, 0.20, 120.0, 1.0, 0.35, 210.0)
  trajSet(link06, MSINE, 8.0, 0.2, 6.0, 0.09, 45.0, 3.0, 0.18, 135.0, 1.5, 0.32, 225.0)
}
wait(8.0)
wait(0.3)


# -----------------------------
# Phase 7: Small inspection "wrist scan" (8s)
# (Low amplitude, low frequency; realistic sensor sweep)
# -----------------------------
parallel(0.0) {
  trajSet(link04, SINE, 8.0, 0.2,  6.0, 0.12, 0.0)
  trajSet(link05, SINE, 8.0, 0.2,  6.0, 0.12, 90.0)
  trajSet(link06, SINE, 8.0, 0.2, 10.0, 0.10, 45.0)
}
wait(8.0)
wait(0.3)

# -----------------------------
# Phase 8: Return HOME (2.5s)
# -----------------------------
parallel(0.0) {
  trajSet(link01, TRAP,   0.0,  60.0, 120.0)
  trajSet(link02, TRAP,  45.0,  50.0, 100.0)
  trajSet(link03, TRAP, -60.0,  50.0, 100.0)
  trajSet(link04, TRAP,   0.0,  60.0, 120.0)
  trajSet(link05, TRAP,   0.0,  60.0, 120.0)
  trajSet(link06, TRAP,   0.0,  80.0, 160.0)
}
wait(2.5)

trajClear()
wait(0.5)
stop()
