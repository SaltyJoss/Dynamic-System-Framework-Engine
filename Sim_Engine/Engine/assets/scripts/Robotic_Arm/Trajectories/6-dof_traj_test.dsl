# ------------------------------------------------------------
# Trajectory test: 6 joints (TRAP + SINE + MSINE mix)
# Assumes Z1 has joints/links named: link00..link05
# ------------------------------------------------------------

load(robot, Z1)

wait(2.0)

trajClear()
start()

# settle
wait(2.0)

# Run all 6 together for 12 seconds
parallel(10.0) {

    # J1: TRAP -> move to -60deg
    trajSet(link01, TRAP, -1.0471975512, 0.9, 1.8)

    # J2: SINE -> amp 0.30rad, 0.40Hz, 12s, phase 0
    trajSet(link02, SINE, 0.30, 0.40, 12.0, 0.0)

    # J3: SINE -> amp 0.20rad, 0.75Hz, 12s, phase pi/2
    trajSet(link03, SINE, 0.20, 0.75, 12.0, 1.57079632679)

    # J4: MSINE -> 12s, (amp,f,phase) x3
    trajSet(link04, MSINE, 12.0, 0.08, 0.30, 0.0, 0.06, 0.90, 1.0, 0.04, 1.60, 2.0)

    # J5: MSINE -> 12s, (amp,f,phase) x2
    trajSet(link05, MSINE, 12.0, 0.10, 0.25, 0.0, 0.05, 1.20, 1.7)
}

# cleanup + settle
trajClear()
wait(1.0)

stop()
