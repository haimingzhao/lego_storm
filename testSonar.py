from robot import robot

robot= robot(0,0,0)
while True:
	rounds = float(input("Enter number of angle : "))
	robot.sonarTurnLeftDeg(rounds)

robot.terminate()
