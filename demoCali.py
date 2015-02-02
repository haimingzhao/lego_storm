from robot import robot
import time

robot = robot()

robot.forward(40)
time.sleep(5)

robot.backward(40)
time.sleep(5)

robot.turnLeft90()

time.sleep(5)

robot.turnRight90()
