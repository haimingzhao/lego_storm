from robot import robot
import time

robot = robot()

port = 0 # port which ultrasoic sensor is plugged in to

robot.sensorEnableUltrasonic(port)

while True:
	# usReading[0] is distance away
	# usReading[1] is timestamp
	usReading = robot.getSensorValue(port)

	if usReading :
		print usReading
	else:
		print "Failed US reading"

	time.sleep(0.05)

robot.terminate()
