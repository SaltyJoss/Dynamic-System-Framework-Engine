# ============================================================
# H1 HUMANOID WALKING - FULL COORDINATED GAIT
#
# Logic: 
#   - Right Leg Phase = Left Leg Phase + 0.5
#   - Left Arm Phase  = Right Leg Phase (Counter-swing)
#   - Right Arm Phase = Left Leg Phase (Counter-swing)
#
# Created by: SaltyJoss
# Double Checked using GitHub Copilot
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
 trajSet(left_knee_link,        TRAP,  40.0, 40.0, 80.0)
 trajSet(left_ankle_link,       TRAP, -20.0, 40.0, 80.0)
 trajSet(right_hip_pitch_link, TRAP, -20.0, 40.0, 80.0)
 trajSet(right_knee_link,       TRAP,  40.0, 40.0, 80.0)
 trajSet(right_ankle_link,      TRAP, -20.0, 40.0, 80.0)
}
wait(2.0)

# ------------------------------------------------------------
# Phase 1: Steady Walk (Base Phase 0.55)
# ------------------------------------------------------------
parallel(0.0) {
 # Left Leg & Right Arm (Phase 0.55)
 trajSet(left_hip_pitch_link,  SINE, 10.0, -15.0, 12.0, 0.55)
 trajSet(left_knee_link,        SINE, 10.0,  30.0, 18.0, 0.55)
 trajSet(left_ankle_link,       SINE, 10.0, -10.0, 10.0, 0.55)
 trajSet(right_shoulder_pitch_link, SINE, 10.0, -10.0, 10.0, 0.55)

 # Right Leg & Left Arm (Phase 1.05)
 trajSet(right_hip_pitch_link, SINE, 10.0, -15.0, 12.0, 1.05)
 trajSet(right_knee_link,       SINE, 10.0,  30.0, 18.0, 1.05)
 trajSet(right_ankle_link,      SINE, 10.0, -10.0, 10.0, 1.05)
 trajSet(left_shoulder_pitch_link,  SINE, 10.0,  10.0, 10.0, 1.05)
}
wait(10.0)

# ------------------------------------------------------------
# Phase 2: Dynamic / Higher Cadence (Base Phase 0.95)
# ------------------------------------------------------------
parallel(0.0) {
 # Left Leg (Phase 0.95)
 trajSet(left_hip_pitch_link,  SINE, 6.0, -15.0, 18.0, 0.95)
 trajSet(left_knee_link,        SINE, 6.0,  30.0, 25.0, 0.95)
 trajSet(right_shoulder_pitch_link, SINE, 6.0, -10.0, 15.0, 0.95)

 # Right Leg (Phase 1.45)
 trajSet(right_hip_pitch_link, SINE, 6.0, -15.0, 18.0, 1.45)
 trajSet(right_knee_link,       SINE, 6.0,  30.0, 25.0, 1.45)
 trajSet(left_shoulder_pitch_link,  SINE, 6.0,  10.0, 15.0, 1.45)
}
wait(6.0)

# ------------------------------------------------------------
# Phase 3: Endurance / Low Cadence (Base Phase 0.42)
# ------------------------------------------------------------
parallel(0.0) {
 # Left Leg (Phase 0.42)
 trajSet(left_hip_pitch_link,  SINE, 18.0, -15.0, 10.0, 0.42)
 trajSet(left_knee_link,        SINE, 18.0,  30.0, 14.0, 0.42)
 trajSet(right_shoulder_pitch_link, SINE, 18.0, -10.0, 10.0, 0.42)

 # Right Leg (Phase 0.92)
 trajSet(right_hip_pitch_link, SINE, 18.0, -15.0, 10.0, 0.92)
 trajSet(right_knee_link,       SINE, 18.0,  30.0, 14.0, 0.92)
 trajSet(left_shoulder_pitch_link,  SINE, 18.0,  10.0, 10.0, 0.92)
}
wait(18.0)

# ------------------------------------------------------------
# Phase 4: Cooldown to Stop (Base Phase 0.22)
# ------------------------------------------------------------
parallel(0.0) {
 trajSet(left_hip_pitch_link,  SINE, 12.0, -15.0,  6.0, 0.22)
 trajSet(left_knee_link,        SINE, 12.0,  30.0,  8.0, 0.22)
 trajSet(right_hip_pitch_link, SINE, 12.0, -15.0,  6.0, 0.72)
 trajSet(right_knee_link,       SINE, 12.0,  30.0,  8.0, 0.72)
}
wait(12.0)

trajClear()
wait(0.25)
stop()