from robot import robot

robot = robot(0,0,0)

while True:
	x,y = raw_input("Enter x and y coordinates to travel to or ctrl+c to quit").split()
	x = float(x)
	y = float(y)	
	robot.navigateToWaypoint(x,y)
	
robot.terminate()
