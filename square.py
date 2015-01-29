import brickpi 
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
ffg = 255/27
minPWM = 25
kp = 250
ki = 570
kd = 13
width = 15.2

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = ffg
motorParams1.minPWM = minPWM
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = kp
motorParams1.pidParameters.k_i = ki
motorParams1.pidParameters.k_d = kd

motorParams2 = interface.MotorAngleControllerParameters()
motorParams2.maxRotationAcceleration = 6.0
motorParams2.maxRotationSpeed = 12.0
motorParams2.feedForwardGain = ffg
motorParams2.minPWM = minPWM
motorParams2.pidParameters.minOutput = -255
motorParams2.pidParameters.maxOutput = 255
motorParams2.pidParameters.k_p = kp
motorParams2.pidParameters.k_i = ki
motorParams2.pidParameters.k_d = kd

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

def runForLength(length):
	angle = length/2.8
	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

def turnClockwise(radius):
	length = radius*width/2
	angle = length/2.8
	interface.increaseMotorAngleReferences(motors,[angle,-angle])
	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

#while True:
#	length = float(input("Enter a length to rotate (in cm): "))
#	turnClockwise(length)

runForLength(40)
turnClockwise(3.14/2)

runForLength(40)
turnClockwise(3.14/2)

runForLength(40)
turnClockwise(3.14/2)

runForLength(40)
turnClockwise(3.14/2)


interface.terminate()


