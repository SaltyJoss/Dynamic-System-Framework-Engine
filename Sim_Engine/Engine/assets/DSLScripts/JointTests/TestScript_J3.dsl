# ============================================================
# JOINT-03 DAMPING/FRICTION INDENTIFICATION
# - Designed to identify the correct coefficients for this joint
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
#
# Gravity = 0 m/s^2
# ============================================================

load(robot, Z1)
set(integrator, rk4)
wait(2.0) # reduces visual stutters

start() 

# Home pose first 
parallel(0.0) { 
	trajSet(link01, TRAP,   0.0, 60.0, 120.0)
	trajSet(link02, TRAP,  45.0, 60.0, 120.0)
	trajSet(link03, TRAP, -60.0, 60.0, 120.0)
	trajSet(link04, TRAP,   0.0, 60.0, 120.0)
	trajSet(link05, TRAP,   0.0, 60.0, 120.0)
	trajSet(link06, TRAP,   0.0, 60.0, 120.0)
}

wait(3.0)
trajClear()
wait(0.25)

# --- Speed set 1 (slow) ---
trajSet(link03, TRAP, -90.0, 30.0, 60.0)
wait(2.0)
trajSet(link03, TRAP, -30.0, 30.0, 60.0)
wait(2.0)

trajClear()
wait(0.25)

# --- Speed set 2 (medium) ---
trajSet(link03, TRAP, -90.0, 60.0, 120.0)
wait(1.5)
trajSet(link03, TRAP, -30.0, 60.0, 120.0)
wait(1.5)

trajClear()
wait(0.25)

# --- Speed set 3 (fast) ---
trajSet(link03, TRAP, -90.0, 120.0, 240.0)
wait(1.2)
trajSet(link03, TRAP, -30.0, 120.0, 240.0)
wait(1.2)

trajClear()
wait(0.25)
stop()