# ---------------------------------------------------------
# trajSet() TEST SUITE
# - Tests TRAP, SINE, and MSINE on each link
# - Motions are small + safe
# - Pauses between tests for clarity
# ---------------------------------------------------------

load(robot, Z1)
set(integrator, rk4)
wait(2)

start()

# ---------------------------------------------------------
# 1) TRAP TESTS
# ---------------------------------------------------------

# Format:
# trajSet(link, TRAP, q1(deg), vmax(deg/s), amax(deg/s²))

trajSet(link01, TRAP,  30, 40, 80)
wait(1)
trajSet(link02, TRAP, -20, 30, 60)
wait(1)
trajSet(link03, TRAP,  25, 35, 70)
wait(1)
trajSet(link04, TRAP, -15, 25, 50)
wait(1)
trajSet(link05, TRAP,  20, 30, 60)
wait(1)
trajSet(link06, TRAP, -30, 40, 80)
wait(2)


# ---------------------------------------------------------
# 2) SINE TESTS
# ---------------------------------------------------------

# Format:
# trajSet(link, SINE, amp(deg), freq(Hz), duration(s) [, phase(deg)])

trajSet(link01, SINE, 10, 0.5, 2.0)
wait(0.5)
trajSet(link02, SINE, 15, 0.7, 2.0)
wait(0.5)
trajSet(link03, SINE,  8, 1.0, 2.0)
wait(0.5)
trajSet(link04, SINE, 12, 0.6, 2.0)
wait(0.5)
trajSet(link05, SINE, 10, 0.8, 2.0)
wait(0.5)
trajSet(link06, SINE,  6, 1.2, 2.0)
wait(2)


# ---------------------------------------------------------
# 3) MSINE TESTS
# ---------------------------------------------------------

# Format:
# trajSet(link, MSINE, duration(s), amp1, f1, ph1, amp2, f2, ph2, ...)

# Two sine components per link (simple but clear)
trajSet(link01, MSINE, 2.0, 10, 0.5, 0,  5, 1.0, 90)
wait(0.5)
trajSet(link02, MSINE, 2.0, 12, 0.6, 0,  6, 1.2, 45)
wait(0.5)
trajSet(link03, MSINE, 2.0,  8, 1.0, 0,  4, 1.5, 30)
wait(0.5)
trajSet(link04, MSINE, 2.0, 10, 0.7, 0,  5, 1.3, 60)
wait(0.5)
trajSet(link05, MSINE, 2.0,  9, 0.9, 0,  4, 1.4, 15)
wait(0.5)
trajSet(link06, MSINE, 2.0,  6, 1.1, 0,  3, 1.8, 75)
wait(2)

stop()