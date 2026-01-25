load(robot, Z1)
start()

parallel(2.0) {
  rotateJointTo(link01, 160,  35)
  rotateJointTo(link02, 160, -20)
  rotateJointTo(link03, 160, -30)
}

stop()
