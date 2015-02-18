from robot import robot
import math

robot = robot();

def navigateToWaypoint(x,y):
	[currentX,currentY,currentTheta] = getAvgParticles
	dx = x - currentX
	dy = y - currentY
	alpha = atan2(dy,dx)
	beta = normalise(alpha - theta)
	robot.turn(beta)
	distance = math.hypot(dx, dy)
	robot.drive(distance)


def normalise(angle):
	if angle < -pi:
		return angle + (2 * pi)
	if angle > pi:
		return angle - (2 * pi)
    else :
    	return angle