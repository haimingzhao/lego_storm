from robot import robot

robot = robot()

for a in range(4):
	for b in range(4):
		robot.forward(10)
	robot.turnRight90()

robot.terminate()
