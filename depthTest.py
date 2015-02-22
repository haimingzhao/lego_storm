from robot import robot

r = robot()
l = r.get_loc()
p = (84,30,0)


while True:
	x = float(input("enter x of particle : "))
	y = float(input("enter y of particle : "))
	theta = float(input("enter theta of particle : "))

	m = l.getDepthMeasurement((x,y,theta))


	print m
