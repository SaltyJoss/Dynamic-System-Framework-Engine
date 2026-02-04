# ============================================================
# DSL (Dynamic Systems Library) script — ~1 minute motion
# Robot: Z1
# Scene: human-like “pick up cup, inspect, sip tilt, set down,
#        wipe/gesture, second micro-lift, place, retract”
#
# Assumptions:
# - rotateJointTo(linkXX, omegaDeg, angleDeg) clamps/obeys joint limits.
# - omegaDeg is the commanded speed (deg/s-ish in your sim).
# - link00..link05 correspond to joints 1..6 along the chain.
#
# Joint limits (using my JSON):
# link01(j1): [-150, 150]  (base yaw)
# link02(j2): [   0, 180]  (shoulder pitch)
# link03(j3): [-160,   0]  (elbow)
# link04(j4): [ -80,  80]  (wrist pitch)
# link05(j5): [ -85,  85]  (wrist yaw)
# link06(j6): [-160, 160]  (wrist roll)
# 
# Created by AI & SaltyJxss (im not writing all of this)
# ============================================================

load(robot, Z1)

# ------------------------------------------------------------
# 0) Idle / settle
# ------------------------------------------------------------
rotateJointTo(link01, 60, 0)
rotateJointTo(link02, 60, 90)
rotateJointTo(link03, 60, -70)
rotateJointTo(link04, 60, 0)
rotateJointTo(link05, 60, 0)
rotateJointTo(link06, 60, 0)

rotateJointBy(link02, 18, 2)
rotateJointBy(link03, 18, -3)
rotateJointBy(link04, 18, 1)
rotateJointBy(link03, 18, 3)
rotateJointBy(link02, 18, -2)
rotateJointBy(link04, 18, -1)

rotateJointBy(link01, 16, 2)
rotateJointBy(link01, 16, -3)
rotateJointBy(link01, 16, 1)

# ------------------------------------------------------------
# 1) Turn toward table object + aim
# ------------------------------------------------------------
rotateJointTo(link01, 80, 30)
rotateJointTo(link02, 90, 105)
rotateJointTo(link03, 100, -95)
rotateJointTo(link04, 90, 10)
rotateJointTo(link05, 90, 12)
rotateJointTo(link06, 90, -8)

rotateJointBy(link01, 35, 3)
rotateJointBy(link01, 35, -5)
rotateJointBy(link01, 35, 2)

rotateJointBy(link05, 35, -4)
rotateJointBy(link05, 35, 6)
rotateJointBy(link05, 35, -2)

rotateJointBy(link04, 35, 5)
rotateJointBy(link04, 35, -4)
rotateJointBy(link04, 35, 2)

# ------------------------------------------------------------
# 2) Reach: extend + approach shaping
# ------------------------------------------------------------
rotateJointTo(link02, 120, 120)
rotateJointTo(link03, 140, -120)
rotateJointTo(link04, 120, 18)
rotateJointTo(link05, 120, 18)
rotateJointTo(link06, 120, -12)

rotateJointBy(link02, 55, 4)
rotateJointBy(link03, 55, -6)
rotateJointBy(link04, 55, 3)
rotateJointBy(link05, 55, 2)

rotateJointBy(link02, 40, -2)
rotateJointBy(link03, 40, 3)
rotateJointBy(link04, 40, -2)
rotateJointBy(link05, 40, -3)

rotateJointBy(link02, 30, 1)
rotateJointBy(link03, 30, -2)
rotateJointBy(link04, 30, 1)
rotateJointBy(link05, 30, 1)

# ------------------------------------------------------------
# 3) Contact + "grasp" mimic (compliance)
# ------------------------------------------------------------
rotateJointBy(link06, 45, 10)
rotateJointBy(link06, 45, -12)
rotateJointBy(link06, 45, 9)
rotateJointBy(link06, 45, -7)
rotateJointBy(link06, 45, 4)

rotateJointBy(link05, 40, 4)
rotateJointBy(link04, 40, -3)
rotateJointBy(link05, 40, -6)
rotateJointBy(link04, 40, 4)
rotateJointBy(link05, 40, 2)
rotateJointBy(link04, 40, -2)

# elbow compliance: ensure link03 stays <= 0 always
rotateJointBy(link03, 50, 8)
rotateJointBy(link03, 50, -10)
rotateJointBy(link03, 50, 3)

# ------------------------------------------------------------
# 4) Lift + stabilize
# ------------------------------------------------------------
rotateJointTo(link02, 140, 105)
rotateJointTo(link03, 140, -110)
rotateJointTo(link04, 140, 12)
rotateJointTo(link05, 140, 10)
rotateJointTo(link06, 140, -10)

rotateJointBy(link02, 28, 2)
rotateJointBy(link02, 28, -2)
rotateJointBy(link03, 28, -3)
rotateJointBy(link03, 28, 3)
rotateJointBy(link04, 28, 2)
rotateJointBy(link04, 28, -2)
rotateJointBy(link05, 28, 2)
rotateJointBy(link05, 28, -2)

# ------------------------------------------------------------
# 5) Bring-to-face: retract + align rim
# ------------------------------------------------------------
rotateJointTo(link01, 120, 10)
rotateJointTo(link02, 140, 85)
rotateJointTo(link03, 160, -135)
rotateJointTo(link04, 160, 30)
rotateJointTo(link05, 160, 22)
rotateJointTo(link06, 160, -18)

rotateJointBy(link04, 60, 6)
rotateJointBy(link04, 60, -4)
rotateJointBy(link05, 60, -5)
rotateJointBy(link05, 60, 3)
rotateJointBy(link06, 60, 4)
rotateJointBy(link06, 60, -3)

rotateJointBy(link01, 55, -4)
rotateJointBy(link01, 55, 3)

# ------------------------------------------------------------
# 6) Inspect / rotate slightly
# ------------------------------------------------------------
rotateJointBy(link06, 70, 25)
rotateJointBy(link05, 70, -10)
rotateJointBy(link04, 70, 10)

rotateJointBy(link06, 45, -18)
rotateJointBy(link05, 45, 8)
rotateJointBy(link04, 45, -6)

rotateJointBy(link06, 35, 10)
rotateJointBy(link05, 35, -5)
rotateJointBy(link04, 35, 3)

# ------------------------------------------------------------
# 7) Sip / pour tilt sequence (multi-pulse)
# ------------------------------------------------------------
rotateJointBy(link04, 90, 25)
rotateJointBy(link06, 90, 15)
rotateJointBy(link03, 90, 8)     # still <= 0 (from -135 to -127)
rotateJointBy(link02, 90, -6)

rotateJointBy(link04, 55, 7)
rotateJointBy(link04, 55, -9)
rotateJointBy(link04, 55, 6)
rotateJointBy(link04, 55, -7)
rotateJointBy(link04, 55, 5)

rotateJointBy(link06, 55, 6)
rotateJointBy(link06, 55, -8)
rotateJointBy(link06, 55, 5)
rotateJointBy(link06, 55, -6)
rotateJointBy(link06, 55, 4)

rotateJointTo(link04, 140, 32)
rotateJointTo(link06, 140, -18)

# ------------------------------------------------------------
# 8) Hesitation: lower slightly then re-raise
# ------------------------------------------------------------
rotateJointBy(link02, 60, 6)
rotateJointBy(link03, 60, 8)
rotateJointBy(link04, 60, -10)
rotateJointBy(link05, 60, -4)

rotateJointBy(link02, 45, -5)
rotateJointBy(link03, 45, -6)
rotateJointBy(link04, 45, 8)
rotateJointBy(link05, 45, 3)

# ------------------------------------------------------------
# 9) Place back down: careful landing
# ------------------------------------------------------------
rotateJointTo(link01, 120, 28)
rotateJointTo(link02, 150, 118)
rotateJointTo(link03, 150, -122)
rotateJointTo(link04, 150, 14)
rotateJointTo(link05, 150, 14)
rotateJointTo(link06, 150, -10)

rotateJointBy(link05, 40, 4)
rotateJointBy(link04, 40, -3)
rotateJointBy(link05, 40, -6)
rotateJointBy(link04, 40, 4)
rotateJointBy(link06, 40, 5)
rotateJointBy(link06, 40, -6)
rotateJointBy(link03, 40, 6)
rotateJointBy(link02, 40, -5)

rotateJointBy(link05, 30, 2)
rotateJointBy(link04, 30, -2)
rotateJointBy(link03, 30, 3)
rotateJointBy(link02, 30, -3)

# ------------------------------------------------------------
# 10) Release mimic + small wipe/gesture (empty hand)
# ------------------------------------------------------------
rotateJointBy(link06, 55, 14)
rotateJointBy(link06, 55, -16)
rotateJointBy(link06, 55, 8)

rotateJointBy(link05, 55, -6)
rotateJointBy(link05, 55, 5)

rotateJointTo(link01, 90, 45)
rotateJointTo(link02, 110, 105)
rotateJointTo(link03, 120, -105)
rotateJointTo(link04, 120, -5)
rotateJointTo(link05, 120, 25)
rotateJointTo(link06, 120, 20)

rotateJointBy(link01, 70, -10)
rotateJointBy(link02, 70, 6)
rotateJointBy(link04, 70, 10)
rotateJointBy(link05, 70, -12)

rotateJointBy(link01, 70, 12)
rotateJointBy(link02, 70, -8)
rotateJointBy(link04, 70, -12)
rotateJointBy(link05, 70, 10)

rotateJointTo(link01, 100, 30)
rotateJointTo(link02, 120, 115)
rotateJointTo(link03, 120, -118)
rotateJointTo(link04, 120, 12)
rotateJointTo(link05, 120, 12)
rotateJointTo(link06, 120, -8)

# ------------------------------------------------------------
# 11) Second micro-lift + reposition + set down
# ------------------------------------------------------------
rotateJointBy(link06, 45, 8)
rotateJointBy(link06, 45, -9)
rotateJointBy(link05, 45, 3)
rotateJointBy(link04, 45, -2)

rotateJointBy(link02, 80, -8)
rotateJointBy(link03, 80, -10)
rotateJointBy(link04, 80, 6)

rotateJointBy(link01, 70, -6)
rotateJointBy(link02, 70, 10)
rotateJointBy(link03, 70, 10)    # stays <= 0
rotateJointBy(link04, 70, -8)
rotateJointBy(link05, 70, -5)

rotateJointBy(link06, 50, 10)
rotateJointBy(link06, 50, -12)
rotateJointBy(link06, 50, 6)

# ------------------------------------------------------------
# 12) Retract to idle
# ------------------------------------------------------------
rotateJointTo(link01, 120, 0)
rotateJointTo(link02, 120, 90)
rotateJointTo(link03, 140, -70)
rotateJointTo(link04, 120, 0)
rotateJointTo(link05, 120, 0)
rotateJointTo(link06, 120, 0)

rotateJointBy(link02, 18, 2)
rotateJointBy(link03, 18, -3)
rotateJointBy(link03, 18, 3)
rotateJointBy(link02, 18, -2)