from robot import robot
import math

def navigateToWaypoint(newX, newY):
	particle = robot.get_loc().get_average()
	currentX = particle[0]
	currentY = particle[1]
	theta = particle[2]
	dx = (100 * newX) - currentX
	dy = (100 * newY) - currentY
	alpha = math.degrees(math.atan2(dy,dx))
	beta = normalise(alpha - theta)
	distance = math.hypot(dx, dy)
	robot.turnDeg(beta)
	robot.forward(distance,False)
	print robot.get_loc().get_average()

def normalise(angle):
	if angle < 0:
		return normalise(angle + 360)
	if angle > 360 :
		return normalise(angle - 360)
	else :
		return angle
robot = robot()

while True:
	x,y = raw_input("Enter x and y coordinates to travel to or ctrl+c to quit").split()
	x = float(x)
	y = float(y)	
	navigateToWaypoint(x,y)
	
robot.terminate()
