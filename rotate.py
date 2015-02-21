from robot import robot
import math

robot = robot()

while True:
	rounds = float(input("Enter number of rounds : "))
	robot.turnRightRad(rounds*2*math.pi)

robot.terminate()
