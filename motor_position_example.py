import calibration
import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

calibration.calibrate(interface, motors)

while True:
	
	angle = float(input("Enter a angle to rotate (in radians): "))
	
	interface.startLogging("test.log")
	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

	
interface.stopLogging()
interface.terminate()
