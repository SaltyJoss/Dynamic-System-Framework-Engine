load(robot, Z1)
set(integrator, rk4)
wait(1.0)

trajClear()
start()
wait(1.0)

# Phase A: TRAP to Pose A (run 3.5s)
parallel(0.0) {
  trajSet(link01, TRAP,  -45.0,  80.0, 160.0)
  trajSet(link02, TRAP,   40.0,  70.0, 140.0)
  trajSet(link03, TRAP,  -35.0,  70.0, 140.0)
  trajSet(link04, TRAP,  -25.0,  80.0, 160.0)
  trajSet(link05, TRAP,  -20.0,  80.0, 160.0)
  trajSet(link06, TRAP,   35.0,  90.0, 180.0)
}
wait(3.5)
wait(0.5)

# Phase B: SINE (run 10s)
parallel(0.0) {
  trajSet(link01, SINE,  15.0, 0.18, 10.0,   0.0)
  trajSet(link02, SINE,  12.0, 0.18, 10.0,  40.0)
  trajSet(link03, SINE,  12.0, 0.18, 10.0,  80.0)
  trajSet(link04, SINE,  10.0, 0.22, 10.0, 120.0)
  trajSet(link05, SINE,  10.0, 0.22, 10.0, 160.0)
  trajSet(link06, SINE,  12.0, 0.22, 10.0, 200.0)
}
wait(10.0)
wait(0.5)

# Phase C: MSINE (run 14s)
parallel(0.0) {
  trajSet(link01, MSINE, 14.0, 15.0, 0.10, 0.0,  8.0, 0.35, 110.0,  4.0, 0.80, 200.0)
  trajSet(link02, MSINE, 14.0,  6.0, 0.12, 30.0, 3.0, 0.40, 140.0,  2.0, 0.90, 220.0)
  trajSet(link03, MSINE, 14.0,  6.0, 0.12, 60.0, 3.0, 0.40, 160.0,  2.0, 0.90, 240.0)
  trajSet(link04, MSINE, 14.0,  8.0, 0.16, 0.0,  4.0, 0.55, 100.0,  2.0, 1.10, 190.0)
  trajSet(link05, MSINE, 14.0,  7.0, 0.16, 30.0, 4.0, 0.55, 130.0,  2.0, 1.10, 210.0)
  trajSet(link06, MSINE, 14.0, 10.0, 0.20, 0.0,  5.0, 0.70,  95.0,  3.0, 1.40, 185.0)
}
wait(14.0)
wait(0.5)

# Phase D: TRAP to Pose B (run 4s)
parallel(0.0) {
  trajSet(link01, TRAP,   55.0,  90.0, 180.0)
  trajSet(link02, TRAP,   20.0,  80.0, 160.0)
  trajSet(link03, TRAP,  -60.0,  80.0, 160.0)
  trajSet(link04, TRAP,  -15.0,  90.0, 180.0)
  trajSet(link05, TRAP,   25.0,  90.0, 180.0)
  trajSet(link06, TRAP,  -35.0, 100.0, 200.0)
}
wait(4.0)
wait(0.5)

# Phase E: SINE (run 12s)
parallel(0.0) {
  trajSet(link01, SINE,  20.0, 0.12, 12.0,   0.0)
  trajSet(link02, SINE,   8.0, 0.16, 12.0,  45.0)
  trajSet(link03, SINE,   8.0, 0.16, 12.0,  90.0)
  trajSet(link04, SINE,  10.0, 0.20, 12.0, 135.0)
  trajSet(link05, SINE,  10.0, 0.20, 12.0, 180.0)
  trajSet(link06, SINE,  12.0, 0.24, 12.0, 225.0)
}
wait(12.0)

trajClear()
wait(0.5)
stop()
