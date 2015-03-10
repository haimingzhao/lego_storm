from robot import robot

r = robot(0,0,0)
l = r.get_loc()


while True:
	x = float(input("enter x of particle : "))
	y = float(input("enter y of particle : "))
	theta = float(input("enter theta of particle : "))

	m = l.getDepthMeasurement((x,y,theta))


	print m

robot.terminate()
