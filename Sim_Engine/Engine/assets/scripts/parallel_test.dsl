load(robot, Z1)

wait(2)

start()

parallel(2.0) {
  rotateJointBy(link01, 50, 100)
		rotateJointBy(link03, 37.5, -90)
  rotateJointBy(link02, 35, 100)
}

stop()