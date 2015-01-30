import brickpi 
import time
import math
import calibration

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
ffg = 255/17.0
minPWM = 36
kp = 200
ki = 180
kd = 330
width = 17.08

calibration.calibrate(interface, motors)

def runForLength(length):
	angle = length/2.8
	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			pass
			#print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

def turnClockwise(radius):
	length = radius*width/2
	angle = length/2.8
	interface.increaseMotorAngleReferences(motors,[angle,-angle])
	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			pass
			#print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

while True:
	rnds = float(input("Enter number of rounds : "))
	turnClockwise(rnds*2*math.pi)

#runForLength(40)
#turnClockwise(math.pi/2)

#runForLength(40)
#turnClockwise(math.pi/2)

#runForLength(40)
#turnClockwise(math.pi/2)

#runForLength(40)
#turnClockwise(math.pi/2)


interface.terminate()
