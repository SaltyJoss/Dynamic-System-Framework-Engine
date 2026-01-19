# Scenario run
set(integrator, rk4)

load(obj, "Engine/assets/objects/Shapes/cube.obj")
colour(obj, 0.2, 0.8, 0.9)

rotate({y}, 45, 0, 90)
rotate(obj, 20, 0, 45)

load(robot, "Engine/assets/objects/Robotic_Arm_Models/Z1/Z1.json")
rotate(link01, 15, 0, 30)
rotate(link02, 20, 0, 60)