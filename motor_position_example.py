import brickpi 
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = 255/27.0
motorParams1.minPWM = 25.0
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = 312.0
motorParams1.pidParameters.k_i = 650
motorParams1.pidParameters.k_d = 5

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = 255/27.0
motorParams2.minPWM = 25.0
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = 312.0
motorParams2.pidParameters.k_i = 650
motorParams2.pidParameters.k_d = 5


interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

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
