from robot import robot
import math

robot = robot(0,0,0)

while True:
	rounds = float(input("Enter number of rounds : "))
	robot.turnRightRad(rounds*2*math.pi)

robot.terminate()
