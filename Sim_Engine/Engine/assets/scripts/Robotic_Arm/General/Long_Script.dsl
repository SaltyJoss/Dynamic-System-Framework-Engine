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
# link00(j1): [-150, 150]  (base yaw)
# link01(j2): [   0, 180]  (shoulder pitch)
# link02(j3): [-160,   0]  (elbow)
# link03(j4): [ -80,  80]  (wrist pitch)
# link04(j5): [ -85,  85]  (wrist yaw)
# link05(j6): [-160, 160]  (wrist roll)
# ============================================================

load(robot, Z1)

# ------------------------------------------------------------
# 0) Idle: subtle human "postural noise" (breathing, readiness)
# ------------------------------------------------------------
rotateJointTo(link00, 60,   0)
rotateJointTo(link01, 60,  90)
rotateJointTo(link02, 60, -70)
rotateJointTo(link03, 60,   0)
rotateJointTo(link04, 60,   0)
rotateJointTo(link05, 60,   0)

# breathing sway (tiny, slow)
rotateJointBy(link01, 18,  +2)
rotateJointBy(link02, 18,  -3)
rotateJointBy(link03, 18,  +1)
rotateJointBy(link02, 18,  +3)
rotateJointBy(link01, 18,  -2)
rotateJointBy(link03, 18,  -1)

rotateJointBy(link00, 16,  +2)
rotateJointBy(link00, 16,  -3)
rotateJointBy(link00, 16,  +1)

# ------------------------------------------------------------
# 1) Attention shift: orient toward object area on table
# ------------------------------------------------------------
rotateJointTo(link00, 80, +30)     # turn toward object
rotateJointTo(link01, 90, 105)     # shoulder forward
rotateJointTo(link02,100,-95)      # elbow bend for reach readiness
rotateJointTo(link03, 90, +10)     # wrist pitch slight
rotateJointTo(link04, 90, +12)     # wrist yaw slight
rotateJointTo(link05, 90,  -8)     # roll align

# micro target refinement (eye-hand loop vibes)
rotateJointBy(link00, 35,  +3)
rotateJointBy(link00, 35,  -5)
rotateJointBy(link00, 35,  +2)

rotateJointBy(link04, 35,  -4)
rotateJointBy(link04, 35,  +6)
rotateJointBy(link04, 35,  -2)

rotateJointBy(link03, 35,  +5)
rotateJointBy(link03, 35,  -4)
rotateJointBy(link03, 35,  +2)

# ------------------------------------------------------------
# 2) Reach phase: extend, approach with a controlled wrist
# ------------------------------------------------------------
rotateJointTo(link01,120, 120)     # reach farther
rotateJointTo(link02,140,-120)
rotateJointTo(link03,120, +18)
rotateJointTo(link04,120, +18)
rotateJointTo(link05,120, -12)

# "approach deceleration": small incremental tweaks
rotateJointBy(link01, 55,  +4)
rotateJointBy(link02, 55,  -6)
rotateJointBy(link03, 55,  +3)
rotateJointBy(link04, 55,  +2)

rotateJointBy(link01, 40,  -2)
rotateJointBy(link02, 40,  +3)
rotateJointBy(link03, 40,  -2)
rotateJointBy(link04, 40,  -3)

rotateJointBy(link01, 30,  +1)
rotateJointBy(link02, 30,  -2)
rotateJointBy(link03, 30,  +1)
rotateJointBy(link04, 30,  +1)

# ------------------------------------------------------------
# 3) Contact + "grasp" mimic: compliance oscillations
# ------------------------------------------------------------
# wrist roll “settle”
rotateJointBy(link05, 45,  +10)
rotateJointBy(link05, 45,  -12)
rotateJointBy(link05, 45,  +9)
rotateJointBy(link05, 45,  -7)
rotateJointBy(link05, 45,  +4)

# wrist yaw/pitch “feel” alignment
rotateJointBy(link04, 40,  +4)
rotateJointBy(link03, 40,  -3)
rotateJointBy(link04, 40,  -6)
rotateJointBy(link03, 40,  +4)
rotateJointBy(link04, 40,  +2)
rotateJointBy(link03, 40,  -2)

# elbow compliance (load transfer)
rotateJointBy(link02, 50,  +8)    # slightly open
rotateJointBy(link02, 50,  -10)   # re-engage hold
rotateJointBy(link02, 50,  +3)

# ------------------------------------------------------------
# 4) Lift: raise smoothly, then stabilize under weight
# ------------------------------------------------------------
rotateJointTo(link01,140, 105)
rotateJointTo(link02,140,-110)
rotateJointTo(link03,140, +12)
rotateJointTo(link04,140, +10)
rotateJointTo(link05,140, -10)

# stabilization tremor (tiny)
rotateJointBy(link01, 28,  +2)
rotateJointBy(link01, 28,  -2)
rotateJointBy(link02, 28,  -3)
rotateJointBy(link02, 28,  +3)
rotateJointBy(link03, 28,  +2)
rotateJointBy(link03, 28,  -2)
rotateJointBy(link04, 28,  +2)
rotateJointBy(link04, 28,  -2)

# ------------------------------------------------------------
# 5) Bring-to-face: retract elbow, rotate base inward, align rim
# ------------------------------------------------------------
rotateJointTo(link00,120, +10)
rotateJointTo(link01,140,  85)
rotateJointTo(link02,160,-135)
rotateJointTo(link03,160, +30)
rotateJointTo(link04,160, +22)
rotateJointTo(link05,160, -18)

# mid-course corrections (human continuous adjustment)
rotateJointBy(link03, 60,  +6)
rotateJointBy(link03, 60,  -4)
rotateJointBy(link04, 60,  -5)
rotateJointBy(link04, 60,  +3)
rotateJointBy(link05, 60,  +4)
rotateJointBy(link05, 60,  -3)

rotateJointBy(link00, 55,  -4)
rotateJointBy(link00, 55,  +3)

# ------------------------------------------------------------
# 6) Inspect/rotate slightly (like checking contents)
# ------------------------------------------------------------
rotateJointBy(link05, 70, +25)    # roll to "look"
rotateJointBy(link04, 70, -10)    # yaw a bit
rotateJointBy(link03, 70, +10)    # pitch up slightly

rotateJointBy(link05, 45, -18)
rotateJointBy(link04, 45, +8)
rotateJointBy(link03, 45, -6)

rotateJointBy(link05, 35, +10)
rotateJointBy(link04, 35, -5)
rotateJointBy(link03, 35, +3)

# ------------------------------------------------------------
# 7) Sip / pour tilt sequence (multi-pulse, realistic)
# ------------------------------------------------------------
# primary tilt
rotateJointBy(link03, 90, +25)    # wrist pitch forward
rotateJointBy(link05, 90, +15)    # roll to angle rim
rotateJointBy(link02, 90,  +8)    # open elbow to avoid collision
rotateJointBy(link01, 90,  -6)    # shoulder compensates

# sip pulses (forward/back micro)
rotateJointBy(link03, 55,  +7)
rotateJointBy(link03, 55,  -9)
rotateJointBy(link03, 55,  +6)
rotateJointBy(link03, 55,  -7)
rotateJointBy(link03, 55,  +5)

# small roll pulses (like adjusting lip contact)
rotateJointBy(link05, 55,  +6)
rotateJointBy(link05, 55,  -8)
rotateJointBy(link05, 55,  +5)
rotateJointBy(link05, 55,  -6)
rotateJointBy(link05, 55,  +4)

# recover toward level (don’t spill)
rotateJointTo(link03,140, +32)
rotateJointTo(link05,140, -18)

# ------------------------------------------------------------
# 8) Lower slightly, pause, then re-raise (natural hesitation)
# ------------------------------------------------------------
rotateJointBy(link01, 60,  +6)    # shoulder up a touch (back off)
rotateJointBy(link02, 60,  +8)    # open elbow slightly
rotateJointBy(link03, 60, -10)    # wrist flatter
rotateJointBy(link04, 60,  -4)

rotateJointBy(link01, 45,  -5)    # return
rotateJointBy(link02, 45,  -6)
rotateJointBy(link03, 45,  +8)
rotateJointBy(link04, 45,  +3)

# ------------------------------------------------------------
# 9) Place back down (careful landing with micro alignment)
# ------------------------------------------------------------
rotateJointTo(link00,120, +28)    # turn back to table area
rotateJointTo(link01,150, 118)    # extend
rotateJointTo(link02,150,-122)
rotateJointTo(link03,150, +14)
rotateJointTo(link04,150, +14)
rotateJointTo(link05,150, -10)

# landing micro-adjustments (edge finding)
rotateJointBy(link04, 40,  +4)
rotateJointBy(link03, 40,  -3)
rotateJointBy(link04, 40,  -6)
rotateJointBy(link03, 40,  +4)
rotateJointBy(link05, 40,  +5)
rotateJointBy(link05, 40,  -6)
rotateJointBy(link02, 40,  +6)
rotateJointBy(link01, 40,  -5)

rotateJointBy(link04, 30,  +2)
rotateJointBy(link03, 30,  -2)
rotateJointBy(link02, 30,  +3)
rotateJointBy(link01, 30,  -3)

