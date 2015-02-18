from robot import robot
import math

robot = robot()

for i in range(4):
	for i in range(4):
		robot.forward(10)
	robot.turnRight90()

robot.terminate()
