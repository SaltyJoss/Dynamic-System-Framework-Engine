# ---------------------------
# Robot demo: pick-and-place test (roughly)
# Mix of absolute targets + delta nudges
# Uses:
#   rotateJointTo(link, omegaDegPerSec, targetDeg)
#   rotateJointBy(link, omegaDegPerSec, deltaDeg)
#
# Notes:
# - "To" commands establish repeatable poses.
# - "By" commands add small procedural motion (nudges, grip, flourish).
#
# Created By: SaltyJxss
# ---------------------------

# load the arm
load(robot, Z1)

# ===========================
# 0) Wake / settle (small absolute posture)
# ===========================
rotateJointTo(link01,  40,   0)
rotateJointTo(link02,  40,  10)
rotateJointTo(link03,  40, -20)
rotateJointTo(link04,  50,   0)
rotateJointTo(link05,  50,   0)
rotateJointTo(link06,  80,   0)

# micro-settle (tiny deltas)
rotateJointBy(link02,  25,  -4)
rotateJointBy(link03, -25,   4)
rotateJointBy(link02,  25,   4)
rotateJointBy(link03, -25,  -4)


# ===========================
# 1) Scan left-right (repeatable scan using "To")
# ===========================
rotateJointTo(link01,  50,  35)
rotateJointTo(link02,  35,  12)
rotateJointTo(link03,  35, -18)

rotateJointTo(link01,  50, -35)
rotateJointTo(link02,  35,   8)
rotateJointTo(link03,  35, -22)

rotateJointTo(link01,  50,   0)
rotateJointTo(link02,  35,  10)
rotateJointTo(link03,  35, -20)


# ===========================
# 2) Move to "pre-grasp" pose (absolute targets)
# ===========================
# Shoulder down, elbow out, wrist align
rotateJointTo(link02,  35, -25)
rotateJointTo(link03,  35,  35)
rotateJointTo(link04,  55,  15)
rotateJointTo(link05,  55,  10)
rotateJointTo(link06,  90,   0)


# ===========================
# 3) Approach (delta nudges)
# ===========================
rotateJointBy(link02,  20,  -6)
rotateJointBy(link03, -20,   8)
rotateJointBy(link04,  25,   6)

# tiny "hover correction" (optional wobble)
rotateJointBy(link05,  25,   3)
rotateJointBy(link05,  25,  -3)


# ===========================
# 4) "Grip" (simulate gripper using wrist roll + slight wrist pitch)
# ===========================
# roll in, pitch down a bit, roll back slightly
rotateJointBy(link06,  90,  40)
rotateJointBy(link05,  45, -10)
rotateJointBy(link06,  90, -15)

# lock to a clean absolute wrist state (repeatable)
rotateJointTo(link06,  90,  20)


# ===========================
# 5) Lift (absolute lift posture)
# ===========================
# Keep wrist fairly stable while retracting
rotateJointTo(link03,  35,  10)
rotateJointTo(link02,  35, -10)
rotateJointTo(link04,  45,   5)
rotateJointTo(link05,  45,   5)


# ===========================
# 6) Turn to drop zone (absolute base yaw)
# ===========================
rotateJointTo(link01,  55,  60)


# ===========================
# 7) Place posture (absolute targets, smaller reach)
# ===========================
rotateJointTo(link02,  30, -18)
rotateJointTo(link03,  30,  22)
rotateJointTo(link04,  40,  12)
rotateJointTo(link05,  40,   8)


# ===========================
# 8) "Release" (undo wrist roll; mix by + to)
# ===========================
rotateJointBy(link06,  90, -25)
rotateJointBy(link05,  45,   8)
rotateJointBy(link06,  90,  -5)

# snap wrist back to neutral
rotateJointTo(link06,  90,   0)
rotateJointTo(link05,  45,   0)


# ===========================
# 9) Retreat (clean back-away; mostly absolute)
# ===========================
rotateJointTo(link04,  40,   0)
rotateJointTo(link03,  35,  -5)
rotateJointTo(link02,  35,   5)


# ===========================
# 10) Reset base + return to neutral-ish
# ===========================
rotateJointTo(link01,  55,   0)

rotateJointTo(link02,  40,  10)
rotateJointTo(link03,  40, -20)
rotateJointTo(link04,  50,   0)
rotateJointTo(link05,  50,   0)
rotateJointTo(link06,  90,   0)


# ===========================
# 11) Little flourish (wave-ish) using By, then snap back with To
# ===========================
rotateJointBy(link05,  60,  20)
rotateJointBy(link06,  90,  35)
rotateJointBy(link05,  60, -40)
rotateJointBy(link06,  90, -70)
rotateJointBy(link05,  60,  20)
rotateJointBy(link06,  90,  35)
rotateJointBy(link06,  90, -35)

# finish clean
rotateJointTo(link05,  60,   0)
rotateJointTo(link06,  90,   0)