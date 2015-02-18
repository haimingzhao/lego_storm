import brickpi
import time
import math
from localisation import localisation

class robot:

	# WARNING: IT APPEARS SENSOR PORTS 4 & 5 ARE BROKEN
	# ALSO SEE MAPPINGS BELOW AS THEY ARE WRONG
	# S1 => port 4 => BROKEN
	# S2 => port 1
	# S3 => port 2
	# S4 => port 3
	# S5 => port 5 => BROKEN

	global wheel_radius
	global wheel_motors
	global wheel_separation
	global sonar_motor
	global all_verbose
	global right_touch
	global left_touch
	global sonar
	global bumper_enabled
	global in_recovery
	global wall_follow_distance
	wheel_radius = 2.8
	wheel_motors = [0,1]
	wheel_separation = 15.89
	sonar_motor = 2
	all_verbose = True
	right_touch = 3
	left_touch = 2
	sonar = 1
	bumper_enabled = False
	in_recovery = False
	wall_follow_distance = 30

#############################################################################
########     MAGIC METHODS    ###############################################
#############################################################################

	def __init__(self, draw=False, record=False):
		self.interface = brickpi.Interface()

		self.initialize()

		self.motorEnable(wheel_motors[0])
		self.motorEnable(wheel_motors[1])
		self.motorEnable(sonar_motor)
		
		motorParams = self.interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/22.0
		motorParams.minPWM = 25
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255

		# proportional gain, reduces error
		motorParams.pidParameters.k_p = 270.0
		# integral gain, removes steady_state error
		motorParams.pidParameters.k_i = 400
		# differential gain, reduce settling time
		motorParams.pidParameters.k_d = 160

		self.setMotorAngleControllerParameters(wheel_motors[0], motorParams)
		self.setMotorAngleControllerParameters(wheel_motors[1], motorParams)
		self.setMotorAngleControllerParameters(sonar_motor, motorParams)

		# initialise localisation
		self.loc = localisation(draw, record)

#############################################################################
########     PUBLIC BRICKPI INTERFACE METHODS    ############################
#############################################################################

	# this starts a separate thread that continuously polls the activated sensors
	# as well as controls the motors that were started
	def initialize(self):
		self.interface.initialize()

	# softly stop all motors and sensors, stop the polling and control thread
	def terminate(self):
		self.interface.terminate()

	# immediately stop all motors, stop the polling and control thread
	# BAD FOR HARDWARE - DO NOT DO THIS!!!!!
	def emergencyStop(self):
		self.interface.emergencyStop()

	# start individual motors
	def motorEnable(self, port):
		self.interface.motorEnable(port)

	# stop individual motors
	def motorDisable(self, port):
		self.interface.motorDisable(port)

	# activate individual sensors
	def sensorEnable(self, port, sensor_type):
		self.interface.sensorEnable(port, sensor_type)

	# deactivate individual sensors
	def sensorDisable(self, port):
		self.interface.sensorDisable(port)

	# thread safe access to sensor values
	def getSensorValue(self, port):
		return self.interface.getSensorValue(port)

	# low-level motor interface -- overrides controller
	# useful for instant stop of motor
	def setMotorPwm(self, port, pwm):
		self.interface.setMotorPwm(port, pwm)

	# set the controller parameters to non-default values
	def setMotorAngleControllerParameters(self, motor_port, motor_params):
		self.interface.setMotorAngleControllerParameters(motor_port, motor_params)

	# set a controller speed reference -- overrides low-level setMotorPwm
	def setMotorRotationSpeedReference(self, motor, speed):
		self.interface.setMotorRotationSpeedReference(motor, speed)

	# set controller speed references -- overrides low-level setMotorPwm.
	# this version guarantees synchronous operation
	def setMotorRotationSpeedReferences(self, motors, speeds):
		self.interface.setMotorRotationSpeedReferences(motors, speeds)

	# figure out, if the set speed reference has been reached
	def motorRotationSpeedReferenceReached(self, motor):
		return self.interface.motorRotationSpeedReferenceReached(motor)

	# set a controller angle reference
	def setMotorAngleReference(self, motor, angle):
		self.interface.setMotorAngleReference(motor, angle)

	# set controller speed references
	# this version guarantees synchronous
	def setMotorAnglesReferences(self, motors, angles):
		self.interface.setMotorAnglesReferences(motors, angles)

	# increase a controller angle reference
	def increaseMotorAngleReference(self, motor, angle):
		self.interface.increaseMotorAngleReference(motor, angle)

	# increase controller speed references
	# this version guarantees synchronous
	def increaseMotorAngleReferences(self, motors, angles):
		self.interface.increaseMotorAngleReferences(motors, angles)

	# query the current (actual) motor speeds
	def getMotorAngles(self, motors):
		return self.interface.getMotorAngles(motors)

	# query the current (actual) motor speed
	def getMotorAngle(self, motor):
		return self.interface.getMotorAngle(motor)

	# figure out, if the set angle reference has been reached
	def motorAngleReferencesReached(self, motors):
		return self.interface.motorAngleReferencesReached(motors)

	# figure out, if the set angle reference has been reached
	def motorAngleReferenceReached(self, motor):
		return self.interface.motorAngleReferenceReached(motor)

	def startLogging(self, file_name):
		self.interface.startLogging(file_name)

	def stopLogging(self):
		self.interface.stopLogging()

	def sensorEnableUltrasonic(self, port):
		self.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

	def sensorEnableTouch(self, port):
		self.sensorEnable(port, brickpi.SensorType.SENSOR_TOUCH)


#############################################################################
########     ENVIRONMENT CONTROL METHODS    #################################
#############################################################################

	def setAllVerbose(self, value):
		all_verbose = value

	def getAllVerbose(self):
		return all_verbose

#############################################################################
########     PUBLIC MOVEMENT METHODS    #####################################
#############################################################################

	#distance in cm
	def forward(self, distance, verbose=False):
		self.linearMove(distance)
		self.loc.loc_distance(distance)
		if verbose or all_verbose: print "Completed forward " + str(distance)

	def backward(self, distance, verbose=False):
		self.linearMove(-distance)
		self.loc.loc_distance(-distance)
		if verbose or all_verbose: print "Completed backward " + str(distance)
 
	def turnRightRad(self, radius, verbose=False):
		length = radius*wheel_separation/2
		angle = length/wheel_radius
		self.turn([angle, -angle])
		self.loc.loc_rotation(math.degrees(-radius))
		if verbose or all_verbose: print "Completed right turn " + str(radius)

	def turnLeftRad(self, radius, verbose=False):
		length = radius*wheel_separation/2
		angle = length/wheel_radius
		self.turn([-angle, angle])
		self.loc.loc_rotation(math.degrees(radius))
		if verbose or all_verbose: print "Completed left turn " + str(radius)

	def turnRightDeg(self, degrees):
		self.turnRightRad(math.radians(degrees))

	def turnLeftDeg(self, degrees):
		self.turnLeftRad(math.radians(degrees))

	def turnRight90(self):
		self.turnRightRad(math.pi/2)	

	def turnLeft90(self):
		self.turnLeftRad(math.pi/2)

	def turnDeg(self,degrees):
		if degrees <180 :
			self.turnLeftDeg(degrees)
		else : 
			self.turnRightDeg(degrees)

	def instantStop(self, verbose=False):
		self.setMotorPwm(0, 0)
		self.setMotorPwm(1, 0)
		if verbose or all_verbose: print "Instant stop!!!"


#############################################################################
########     PUBLIC SENSOR METHODS    #######################################
#############################################################################

	def setWallFollowDistance(self, distance):
		global wall_follow_distance
		wall_follow_distance = distance

	def getWallFollowDistance(self):
		return wall_follow_distance

	global pre_dis
	pre_dis = -1
	def wallFollow(self):
		while True:
			self.setMotorRotationSpeedReferences([0,1], [150, 150])
			while True:
				dis = self.getSensorValue(sonar)
				print dis
				if dis > wall_follow_distance or dis < wall_follow_distance:
					self.instantStop()
					break
				time.sleep(0.1)
			if dis > wall_follow_distance:
				self.turnLeftDeg(5)
				self.forward(5)
			else:
				self.turnRightDeg(5)
				self.forward(5)

	def enableBumper(self, verbose=False):
		global bumper_enabled
		self.sensorEnableTouch(left_touch)
		self.sensorEnableTouch(right_touch)
		if verbose or all_verbose: print "Bumper Enabled"
		bumper_enabled = True

	def enableSonar(self, verbose=False):
		self.sensorEnableUltrasonic(sonar)
		if verbose or all_verbose: print "Sonar Enabled"

	def disableBumper(self):
		global bumper_enabled
		self.sensorDisable(left_touch)
		self.sensorDisable(right_touch)
		bumper_enabled = False

	def disableSonar(self):
		self.sensorDisable(sonar)

	def check_bumper(self):
		return self.getSensorValue(left_touch)[0] or self.getSensorValue(right_touch)[0]

	def recover(self):
		global in_recovery
		left = self.getSensorValue(left_touch)[0]
		right = self.getSensorValue(right_touch)[0]
		in_recovery = True
		self.backward(10)
		if left and right:
			print "Recovering from both"
			self.turnRight90()
		elif left:
			print "Recovering from left"
			self.turnRight90()
		elif right:
			print "Recovering from right"
			self.turnLeft90()
		in_recovery = False

#############################################################################
########     LOCALISATION METHODS    ########################################
#############################################################################

	def get_loc(self):
		return self.loc

#############################################################################
########     PRIVATE METHODS    #############################################
#############################################################################

	def wheelRadianTurn(radians):
		return (radians * wheel_seperation) / (2 * wheel_radius)

	def turn(self, angles):
		self.increaseMotorAngleReferences(wheel_motors, angles)
		while not self.motorAngleReferencesReached(wheel_motors):
			motorAngles = self.getMotorAngles(wheel_motors)
			time.sleep(0.1)

	# direction is true if forward, false if backward
	def linearMove(self, distance):
		angle = distance/wheel_radius
		self.increaseMotorAngleReferences(wheel_motors, [angle, angle])
		while not self.motorAngleReferencesReached(wheel_motors):
			if bumper_enabled and self.check_bumper() and not in_recovery:
				self.instantStop()
				self.recover()
				break
			motorAngles = self.getMotorAngles(wheel_motors)
			time.sleep(0.1)


