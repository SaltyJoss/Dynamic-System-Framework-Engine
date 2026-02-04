# ---------------------------
# Z1 FAST demo (settle-friendly)
# - Avoids link06 until the end
# - Fewer commands
# ---------------------------

load(robot, Z1)
set(integrator, rk4)
wait(0.2)

start()

# Wake (small, settle-friendly)
rotateJointBy(link02, 120,  14)
rotateJointBy(link03, 120, -10)
rotateJointBy(link04, 140,  14)

# Big motion (base scan)
rotateJointBy(link01, 160,  60)
rotateJointBy(link01, 160, -120)
rotateJointBy(link01, 160,  60)

# Reach + place (compact)
rotateJointBy(link02, 140, 	20)
rotateJointBy(link03, 140, -25)
rotateJointBy(link04, 160,  12)
rotateJointBy(link05, 160,  10)

# Tiny wrist flourish LAST (so if it crawls, it doesn't waste the whole demo)
rotateJointBy(link06, 180,  10)
rotateJointBy(link06, 180, -10)

stop()
