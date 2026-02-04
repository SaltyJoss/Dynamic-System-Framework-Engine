# ---------------------------------------------
# Trajectory test: TRAP + SINE + MSINE in parallel
# ---------------------------------------------

load(robot, Z1)

trajClear()

start()

# Phase 1: settle at start (5s)
wait(5)

# Phase 2: run trajectories (10s)
parallel(10.0) {

    # Trapezoid: trajSet(link, TRAP, q1, vmax, amax)
    trajSet(link00, TRAP, 1.57079632679, 1.0, 2.0)

    # Sine: trajSet(link, SINE, amp, freqHz, durationSec, phaseRad)
    trajSet(link01, SINE, 0.35, 0.50, 10.0, 0.0)

    # Multisine: trajSet(link, MSINE, durationSec, amp1,f1,ph1, amp2,f2,ph2, ...)
    trajSet(link02, MSINE, 10.0, 0.10, 0.30, 0.0, 0.08, 0.80, 1.2, 0.05, 1.60, 2.4)
}

# Phase 3: clear trajectories and settle (2s)
trajClear()
wait(2)

stop()
