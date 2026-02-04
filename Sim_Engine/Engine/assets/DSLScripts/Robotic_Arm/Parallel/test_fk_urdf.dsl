# rotateJointTo(<link>, <maxOmega>, <angleDeg>)

load(robot, Z1)

wait(5)

start()
rotateJointTo(link01, 15, -97)
rotateJointTo(link02, 25,  45)
rotateJointTo(link03, 10, -50)
rotateJointTo(link04, 5,  -20)
rotateJointTo(link05, 2,   22)
rotateJointTo(link06, 10,   140)
stop()