# ---------------------------
# Robot demo: pick-and-place test (roughly) (delta moves)
# Uses: rotateJointBy(link, omegaDegPerSec, deltaDeg)
#
# Created By: SaltyJxss
# ---------------------------

# Load Robotic Arm
load(robot, Z1)

wait(1)

start()

# Wake / settle
rotateJointBy(link01,  25,  10)
rotateJointBy(link02,  25, -10)
rotateJointBy(link03,  25,   8)
rotateJointBy(link04,  25,   5)
rotateJointBy(link05,  25,   5)
rotateJointBy(link06,  25,   8)

# Scan left-right (base yaw), with small shoulder/elbow compensation
rotateJointBy(link01,  35,  35)
rotateJointBy(link02,  30,  -8)
rotateJointBy(link03,  30,   6)
rotateJointBy(link01,  35, -70)
rotateJointBy(link02,  30,   8)
rotateJointBy(link03,  30,  -6)
rotateJointBy(link01,  35,  35)

# Reach toward "object" (shoulder down, elbow out, wrist align)
rotateJointBy(link02,  30, -35)
rotateJointBy(link03,  35,  55)
rotateJointBy(link04,  45,  20)
rotateJointBy(link05,  45,  10)

# Fine approach (tiny nudges)
rotateJointBy(link02,  20,  -6)
rotateJointBy(link03,  20,   8)
rotateJointBy(link04,  25,   6)

# "Grip" (simulate gripper by wrist roll + slight wrist pitch)
rotateJointBy(link06,  80,  40)
rotateJointBy(link05,  40, -10)
rotateJointBy(link06,  80, -15)

# Lift (reverse some reach, keep wrist stable)
rotateJointBy(link03,  30, -35)
rotateJointBy(link02,  30,  25)
rotateJointBy(link04,  40, -10)

# Turn toward "drop zone"
rotateJointBy(link01,  35,  60)

# Place posture (reach forward but less than before)
rotateJointBy(link02,  25, -20)
rotateJointBy(link03,  30,  28)
rotateJointBy(link04,  35,  12)

# "Release" (undo wrist roll)
rotateJointBy(link06,  80, -25)
rotateJointBy(link05,  40,   8)
rotateJointBy(link06,  80,  -5)

# Retreat (back away cleanly)
rotateJointBy(link04,  35, -10)
rotateJointBy(link03,  30, -20)
rotateJointBy(link02,  25,  15)

# Reset base
rotateJointBy(link01, 35, -60)

# Return to neutral-ish
rotateJointBy(link02,  30,  20)
rotateJointBy(link03,  30, -30)
rotateJointBy(link04,  40, -10)
rotateJointBy(link05,  40, -10)
rotateJointBy(link06,  60,  -5)

# Little flourish at the end (wave-ish)
rotateJointBy(link05,  60,  20)
rotateJointBy(link06,  90,  35)
rotateJointBy(link05,  60, -40)
rotateJointBy(link06,  90, -70)
rotateJointBy(link05,  60,  20)
rotateJointBy(link06,  90,  35)
rotateJointBy(link06,  90, -35)

wait(1)

stop()