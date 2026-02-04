# ---------------------------------------------------------
# REALISTIC MOVEMENT DEMO
# ---------------------------------------------------------

load(robot, Z1)
set(integrator, rk4)
wait(2)

start()

# ---------------------------------------------------------
# 1) IDLE BREATHING (subtle MSINE)
# ---------------------------------------------------------
parallel(3.0) {
    # Shoulder + elbow micro‑motion
    trajSet(link02, MSINE, 3.0, 3, 0.3, 0,  1, 0.8, 90)
    trajSet(link03, MSINE, 3.0, 2, 0.4, 0,  1, 1.0, 45)

    # Wrist tiny oscillation
    trajSet(link05, MSINE, 3.0, 1, 1.2, 0,  0.5, 2.0, 30)
}

wait(0.5)


# ---------------------------------------------------------
# 2) POSTURE SHIFT (TRAP)
# ---------------------------------------------------------
parallel(2.0) {
    trajSet(link01, TRAP,  25, 40, 80)   # base rotates slightly
    trajSet(link02, TRAP, -15, 35, 70)   # shoulder dips
    trajSet(link03, TRAP,  10, 30, 60)   # elbow adjusts
}

wait(0.5)


# ---------------------------------------------------------
# 3) LOOK AROUND (SINE + TRAP)
# ---------------------------------------------------------
parallel(3.0) {
    # Base oscillates like scanning
    trajSet(link01, SINE, 15, 0.4, 3.0)

    # Neck/wrist subtle motion
    trajSet(link05, SINE,  8, 1.0, 3.0)

    # Head tilt (TRAP)
    trajSet(link04, TRAP, -10, 25, 50)
}

wait(0.5)


# ---------------------------------------------------------
# 4) GESTURE (TRAP + MSINE flourish)
# ---------------------------------------------------------
parallel(3.0) {
    # Arm extends
    trajSet(link02, TRAP,  20, 40, 80)
    trajSet(link03, TRAP, -25, 35, 70)

    # Wrist flourish (complex MSINE)
    trajSet(link06, MSINE, 3.0, 8, 1.0, 0,  4, 2.0, 90)
}

wait(0.5)


# ---------------------------------------------------------
# 5) RETURN TO NEUTRAL (TRAP)
# ---------------------------------------------------------
parallel(2.5) {
    trajSet(link01, TRAP, 0, 40, 80)
    trajSet(link02, TRAP, 0, 40, 80)
    trajSet(link03, TRAP, 0, 40, 80)
    trajSet(link04, TRAP, 0, 40, 80)
    trajSet(link05, TRAP, 0, 40, 80)
    trajSet(link06, TRAP, 0, 40, 80)
}

stop()
