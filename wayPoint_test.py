from robot import robot
import math

robot = robot();

def navigateToWaypoint(x,y):
	particle = getAvgParticles
	dx = x - currentX
	dy = y - currentY
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