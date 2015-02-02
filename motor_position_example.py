from robot import robot
import time

robot = robot()

motors = [0,1]

while True:
	angle = float(input("Enter a angle to rotate (in radians): "))
	
	robot.startLogging("test.log")
	robot.increaseMotorAngleReferences(motors,[angle,angle])

	while not robot.motorAngleReferencesReached(motors):
		motorAngles = robot.getMotorAngles(motors)
		if motorAngles:
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Completed turn"

robot.stopLogging()
robot.terminate()
