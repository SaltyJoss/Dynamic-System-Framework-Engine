# ============================================================
# H1 HUMANOID WALKING SHOWCASE (EXTENDED)
#
# Gait: quasi-static sinusoidal walk
# Units: degrees (engine converts internally)
#
# Created by: SaltyJoss
# ============================================================

load(robot, H1)
set(integrator, rk4)
wait(2.0)

trajClear()
wait(0.25)

start()
wait(0.5)

# ------------------------------------------------------------
# Phase 0: Neutral standing pose
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(torso_link, TRAP, 0.0, 60.0, 120.0)
 trajSet(left_hip_pitch_link,  TRAP, -20.0, 40.0, 80.0)
 trajSet(left_knee_link,       TRAP,  40.0, 40.0, 80.0)
 trajSet(left_ankle_link,      TRAP, -20.0, 40.0, 80.0)
 trajSet(right_hip_pitch_link, TRAP, -20.0, 40.0, 80.0)
 trajSet(right_knee_link,      TRAP,  40.0, 40.0, 80.0)
 trajSet(right_ankle_link,     TRAP, -20.0, 40.0, 80.0)
}
wait(2.0)
wait(0.3)

# ------------------------------------------------------------
# Phase 1a: Walking gait (steady)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 10.0, -15.0, 12.0, 0.55)
 trajSet(left_knee_link,       SINE, 10.0,  30.0, 18.0, 0.55)
 trajSet(left_ankle_link,      SINE, 10.0, -10.0, 10.0, 0.55)

 trajSet(right_hip_pitch_link, SINE, 10.0, -15.0, 12.0, 0.55)
 trajSet(right_knee_link,      SINE, 10.0,  30.0, 18.0, 0.55)
 trajSet(right_ankle_link,     SINE, 10.0, -10.0, 10.0, 0.55)

 trajSet(left_shoulder_pitch_link,  SINE, 10.0,  10.0, 10.0, 0.55)
 trajSet(left_elbow_link,           SINE, 10.0,  20.0, 15.0, 0.55)

 trajSet(right_shoulder_pitch_link, SINE, 10.0, -10.0, 10.0, 0.55)
 trajSet(right_elbow_link,          SINE, 10.0,  20.0, 15.0, 0.55)
}
wait(10.0)
wait(0.25)


# ------------------------------------------------------------
# Phase 1b
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 10.0, -15.0, 14.0, 0.60)
 trajSet(left_knee_link,       SINE, 10.0,  30.0, 20.0, 0.60)
 trajSet(right_hip_pitch_link, SINE, 10.0, -15.0, 14.0, 0.60)
 trajSet(right_knee_link,      SINE, 10.0,  30.0, 20.0, 0.60)
 trajSet(left_shoulder_pitch_link,  SINE, 10.0,  10.0, 12.0, 0.60)
 trajSet(right_shoulder_pitch_link, SINE, 10.0, -10.0, 12.0, 0.60)
}
wait(10.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 1c
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 12.0, -15.0, 12.0, 0.45)
 trajSet(left_knee_link,       SINE, 12.0,  30.0, 16.0, 0.45)
 trajSet(right_hip_pitch_link, SINE, 12.0, -15.0, 12.0, 0.45)
 trajSet(right_knee_link,      SINE, 12.0,  30.0, 16.0, 0.45)
 trajSet(left_elbow_link,      SINE, 12.0,  20.0, 10.0, 0.45)
 trajSet(right_elbow_link,     SINE, 12.0,  20.0, 10.0, 0.45)
}
wait(12.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 2b
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 6.0, -15.0, 18.0, 0.95)
 trajSet(left_knee_link,       SINE, 6.0,  30.0, 25.0, 0.95)
 trajSet(right_hip_pitch_link, SINE, 6.0, -15.0, 18.0, 0.95)
 trajSet(right_knee_link,      SINE, 6.0,  30.0, 25.0, 0.95)
}
wait(6.0)
wait(0.3)


# ------------------------------------------------------------
# Phase 4: Long steady walk
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 12.0, -15.0, 12.0, 0.55)
 trajSet(left_knee_link,       SINE, 12.0,  30.0, 18.0, 0.55)
 trajSet(left_ankle_link,      SINE, 12.0, -10.0, 10.0, 0.55)

 trajSet(right_hip_pitch_link, SINE, 12.0, -15.0, 12.0, 0.55)
 trajSet(right_knee_link,      SINE, 12.0,  30.0, 18.0, 0.55)
 trajSet(right_ankle_link,     SINE, 12.0, -10.0, 10.0, 0.55)
}
wait(12.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 5: Slight stride increase (fatigue-free progression)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 14.0, -15.0, 14.0, 0.60)
 trajSet(left_knee_link,       SINE, 14.0,  30.0, 20.0, 0.60)
 trajSet(right_hip_pitch_link, SINE, 14.0, -15.0, 14.0, 0.60)
 trajSet(right_knee_link,      SINE, 14.0,  30.0, 20.0, 0.60)
 trajSet(left_shoulder_pitch_link,  SINE, 14.0,  10.0, 12.0, 0.60)
 trajSet(right_shoulder_pitch_link, SINE, 14.0, -10.0, 12.0, 0.60)
}
wait(14.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 6: Long endurance walk (low cadence, very stable)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 18.0, -15.0, 10.0, 0.42)
 trajSet(left_knee_link,       SINE, 18.0,  30.0, 14.0, 0.42)
 trajSet(right_hip_pitch_link, SINE, 18.0, -15.0, 10.0, 0.42)
 trajSet(right_knee_link,      SINE, 18.0,  30.0, 14.0, 0.42)
 trajSet(left_elbow_link,      SINE, 18.0,  20.0, 10.0, 0.42)
 trajSet(right_elbow_link,     SINE, 18.0,  20.0, 10.0, 0.42)
}
wait(18.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 7: Short dynamic push (tests recovery)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 6.0, -15.0, 18.0, 1.05)
 trajSet(left_knee_link,       SINE, 6.0,  30.0, 26.0, 1.05)
 trajSet(right_hip_pitch_link, SINE, 6.0, -15.0, 18.0, 1.05)
 trajSet(right_knee_link,      SINE, 6.0,  30.0, 26.0, 1.05)
}
wait(6.0)
wait(0.3)

# ------------------------------------------------------------
# Phase 8: Cooldown walk (prepare for stop)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 10.0, -15.0, 10.0, 0.35)
 trajSet(left_knee_link,       SINE, 10.0,  30.0, 12.0, 0.35)
 trajSet(right_hip_pitch_link, SINE, 10.0, -15.0, 10.0, 0.35)
 trajSet(right_knee_link,      SINE, 10.0,  30.0, 12.0, 0.35)
}
wait(10.0)
wait(0.25)

# ------------------------------------------------------------
# Phase 9: Final deceleration walk (approach stop)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 12.0, -15.0,  6.0, 0.22)
 trajSet(left_knee_link,       SINE, 12.0,  30.0,  8.0, 0.22)
 trajSet(right_hip_pitch_link, SINE, 12.0, -15.0,  6.0, 0.22)
 trajSet(right_knee_link,      SINE, 12.0,  30.0,  8.0, 0.22)
}
wait(12.0)
trajClear()

wait(0.25)
stop()