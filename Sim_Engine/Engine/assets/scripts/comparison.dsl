load(robot, Z1)

wait(5)
set(integrator, euler)

start()

parallel(2.0) {
  rotateJointBy(link01, 50, 100)
		rotateJointBy(link03, 37.5, -90)
  rotateJointBy(link02, 35, 100)
}

stop()

load(robot, Z1)
set(integrator, rk4)

wait(5)

start()

parallel(2.0) {
  rotateJointBy(link01, 50, 100)
		rotateJointBy(link03, 37.5, -90)
  rotateJointBy(link02, 35, 100)
}

stop()