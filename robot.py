import brickpi
import time
import math

class robot:

	global motors
	global width
	global wheel_radius
	wheel_radius = 2.8
	motors = [0,1]
	width = 17.05

#############################################################################
########     MAGIC METHODS    ###############################################
#############################################################################

	def __init__(self):
		self.interface = brickpi.Interface()
		self.interface.initialize()

		self.interface.motorEnable(motors[0])
		self.interface.motorEnable(motors[1])
		
		motorParams = self.interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/17.0
		motorParams.minPWM = 36.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		motorParams.pidParameters.k_p = 200.0
		motorParams.pidParameters.k_i = 180
		motorParams.pidParameters.k_d = 330

		self.interface.setMotorAngleControllerParameters(motors[0], motorParams)
		self.interface.setMotorAngleControllerParameters(motors[1], motorParams)
	

#############################################################################
########     PUBLIC INTERFACE METHODS    ####################################
#############################################################################


	def startLogging(self, file_name):
		self.interface.startLogging(file_name)

	def stopLogging(self):
		self.interface.stopLogging()

	def terminate(self):
		self.interface.terminate()

	def setMotorRotationSpeedReferences(self, motor, speed):
		self.interface.setMotorRotationSpeedReferences(motor, speed)

	def motorAngleReferencesReached(self, motors):
		return self.interface.motorAngleReferencesReached(motors)

	def increaseMotorAngleReferences(self, motors, angles):
		self.interface.increaseMotorAngleReferences(motors, angles)

	def getMotorAngles(self, motors):
		return self.interface.getMotorAngles(motors)

	def sensorEnableUltrasonic(self, port):
		self.interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

	def sensorEnableTouch(self, port):
		self.interface.sensorEnable(port, brickpi.SensorType.SENSOR_TOUCH)

	def getSensorValue(self, port):
		return self.interface.getSensorValue(port)


#############################################################################
########     PUBLIC MOVEMENT METHODS    #####################################
#############################################################################

	#distance in cm
	def forward(self, distance):
		self.linearMove(distance)
		print "Completed forward " + str(distance)

	def backward(self, distance):
		self.linearMove(-distance)
		print "Completed backward " + str(distance)
 
	def turnRightRad(self, radius):
		length = radius*width/2
		angle = length/wheel_radius
		self.turn([angle, -angle])
		print "Completed right turn " + str(radius)

	def turnLeftRad(self, radius):
		length = radius*width/2
		angle = length/wheel_radius
		self.turn([-angle, angle])
		print "Completed left turn " + str(radius)

	def turnRightDeg(self, degrees):
		self.turnRightRad(degrees * math.pi / 180)

	def turnLeftDeg(self, degrees):
		self.turnLeftRad(degrees * math.pi / 180)

	def turnRight90(self):
		self.turnRightRad(math.pi/2)	

	def turnLeft90(self):
		self.turnLeftRad(math.pi/2)


#############################################################################
########     PRIVATE METHODS    #############################################
#############################################################################

	def turn(self, angles):
		self.interface.increaseMotorAngleReferences(motors, angles)
		while not self.interface.motorAngleReferencesReached(motors):
			motorAngles = self.interface.getMotorAngles(motors)
			time.sleep(0.1)

	# direction is true if forward, false if backward
	def linearMove(self, distance):
		angle = distance/wheel_radius
		self.interface.increaseMotorAngleReferences(motors, [angle, angle])
		while not self.interface.motorAngleReferencesReached(motors):
			motorAngles = self.interface.getMotorAngles(motors)
			time.sleep(0.1)


