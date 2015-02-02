from robot import robot 
import time
import math

robot = robot()

while True:
	rnds = float(input("Enter number of rounds : "))
	robot.turnRightRad(rnds*2*math.pi)

robot.terminate()
