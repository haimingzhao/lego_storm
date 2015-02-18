from robot import robot
import math

robot = robot()
while True:
	x,y = raw_input("Enter x and y coordinates to travel to or ctrl+c to quit").split()
	x = float(x)
	y = float(y)
	navigateToWaypoint(x,y)
robot.terminate()

def navigateToWaypoint(x,y):
	particle = robot.get_loc().getAverage(robot)
	currentX = particle[0]
	currentY = particle[1]
	theta = particle[3]
	dx = (100 *x) - currentX
	dy = (100 *y) - currentY
	alpha = atan2(dy,dx)
	beta = normalize(alpha - theta,-math.pi,math.pi)
	distance = math.hypot(dx, dy)
	robot.turnDeg(beta)
	robot.forward(self,distance,false)


def normalise(angle):
	if angle < -math.pi:
		return angle + (2 * math.pi)
	if angle > math.pi:
		return angle - (2 * math.pi)
	else :
		return angle