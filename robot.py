import brickpi
import time
import math

class robot:

	global wheel_motors
	global wheel_seperation
	global wheel_radius
	global sonar_motor
	global all_verbose
	wheel_radius = 2.8
	wheel_motors = [0,1]
	wheel_seperation = 17.05
	sonar_motor = 2
	all_verbose = True

#############################################################################
########     MAGIC METHODS    ###############################################
#############################################################################

	def __init__(self):
		self.interface = brickpi.Interface()

		# starts separate thread that continuously polls the activated sensors
		# as well as controls the motors that were started
		self.interface.initialize()

		# activate individual sensors
		self.interface.motorEnable(wheel_motors[0])
		self.interface.motorEnable(wheel_motors[1])
		self.interface.motorEnable(sonar_motor)
		
		motorParams = self.interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/22.0
		motorParams.minPWM = 37.5
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255

		#motorParams.pidParameters.k_p = 350.0
		#motorParams.pidParameters.k_i = 650
		#motorParams.pidParameters.k_d = 50
		# proportional gain, reduces error
		motorParams.pidParameters.k_p = 450.0
		# integral gain, removes steady_state error
		motorParams.pidParameters.k_i = 700
		# differential gain, reduce settling time
		motorParams.pidParameters.k_d = 160

		self.interface.setMotorAngleControllerParameters(wheel_motors[0], motorParams)
		self.interface.setMotorAngleControllerParameters(wheel_motors[1], motorParams)
		self.interface.setMotorAngleControllerParameters(sonar_motor, motorParams)
	

#############################################################################
########     PUBLIC INTERFACE METHODS    ####################################
#############################################################################


	def startLogging(self, file_name):
		self.interface.startLogging(file_name)

	def stopLogging(self):
		self.interface.stopLogging()

	# softly stop all motors and sensors, stop the polling and control threads
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

	def setAllVerbose(self, value):
		all_verbose = value


#############################################################################
########     PUBLIC MOVEMENT METHODS    #####################################
#############################################################################

	#distance in cm
	def forward(self, distance, verbose=False):
		self.linearMove(-distance)
		if verbose or all_verbose: print "Completed forward " + str(distance)

	def backward(self, distance, verbose=False):
		self.linearMove(distance)
		if verbose or all_verbose: print "Completed backward " + str(distance)
 
	def turnRightRad(self, radius):
		length = radius*wheel_separation/2
		angle = length/wheel_radius
		self.turn([-angle, angle])
		if verbose or all_verbose: print "Completed right turn " + str(radius)

	def turnLeftRad(self, radius):
		length = radius*wheel_separation/2
		angle = length/wheel_radius
		self.turn([angle, -angle])
		if verbose or all_verbose: print "Completed left turn " + str(radius)

	def turnRightDeg(self, degrees):
		self.turnRightRad(degreeToRad(degrees))

	def turnLeftDeg(self, degrees):
		self.turnLeftRad(degreeToRad(degrees))

	def turnRight90(self):
		self.turnRightRad(math.pi/2)	

	def turnLeft90(self):
		self.turnLeftRad(math.pi/2)

	def instantStop(self, verbose=False):
		self.interface.setMotorPwm(0, 0)
		self.interface.setMotorPwm(1, 0)
		if verbose or all_verbose: print "Instant stop!!!"

	# immediately stop all motors, stop the polling and control thread
	# BAD FOR HARDWARE - DO NOT USE!!!
	def emergencyStop(self):
		self.interface.emergencyStop()


#############################################################################
########     PUBLIC SENSOR METHODS    #######################################
#############################################################################

	def sonarTurnRight(self, degrees):
		self.interface.increaseMotorAngleReference(sonar_motor, angles)
                while not self.interface.motorAngleReferencesReached(wheel_motors):
                        motorAngles = self.interface.getMotorAngles(wheel_motors)
                        time.sleep(0.1)

	def sonarFront(self):
		pass

	def sonarRightFollow(self):
		pass

	def sonarLeftFollow(self):
		pass

#############################################################################
########     PRIVATE METHODS    #############################################
#############################################################################

	def wheelRadianTurn(radians):
		return (radians * wheel_seperation) / (2 * wheel_radius)

	def degreeToRad(degree):
		return degree * math.pi / 180

	def turn(self, angles):
		self.interface.increaseMotorAngleReferences(wheel_motors, angles)
		while not self.interface.motorAngleReferencesReached(wheel_motors):
			motorAngles = self.interface.getMotorAngles(wheel_motors)
			time.sleep(0.1)

	# direction is true if forward, false if backward
	def linearMove(self, distance):
		angle = distance/wheel_radius
		self.interface.increaseMotorAngleReferences(wheel_motors, [angle, angle])
		while not self.interface.motorAngleReferencesReached(wheel_motors):
			motorAngles = self.interface.getMotorAngles(wheel_motors)
			time.sleep(0.1)


