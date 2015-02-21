import sys
import math
sys.path.insert(0, '../')
from robot import robot

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

robot.enableBumper()

# remember to change origin in localisation to (84,30)

navigateToWaypoint(180, 30)
navigateToWaypoint(180, 54)
navigateToWaypoint(126, 54)
navigateToWaypoint(126, 168)
navigateToWaypoint(126, 126)
navigateToWaypoint(30, 54)
navigateToWaypoint(84, 54)
navigateToWaypoint(84, 30)


robot.terminate()