from robot import robot
import math

def navigateToWaypoint(x,y):
	particle = robot.get_loc().get_Average()
	currentX = particle[0]
	currentY = particle[1]
	theta = particle[2]
	dx = (100 *x) - currentX
	dy = (100 *y) - currentY
	alpha = math.atan2(dy,dx)
	beta = normalise(alpha - theta)
	print beta
	distance = math.hypot(dx, dy)
	robot.turnDeg(beta)
	robot.forward(distance,false)


def normalise(angle):
	if angle < -180:
		return angle + (360)
	if angle > 180:
		return angle - (360)
	else :
		return angle
robot = robot()
while True:
	x,y = raw_input("Enter x and y coordinates to travel to or ctrl+c to quit").split()
	x = float(x)
	y = float(y)
	navigateToWaypoint(x,y)
robot.terminate()
