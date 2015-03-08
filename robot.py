import brickpi
import time
import math
from localisation import localisation
from wallMap import WallMap

class robot:
    # WARNING: IT APPEARS SENSOR PORTS 4 & 5 ARE BROKEN
    # ALSO SEE MAPPINGS BELOW AS THEY ARE WRONG
    # S1 => port 4 => BROKEN
    # S2 => port 1
    # S3 => port 2
    # S4 => port 3
    # S5 => port 5 => BROKEN

    wheel_radius = 2.8
    wheel_motors = [0, 1]
    wheel_separation = 15.75
    sonar_motor = 2
    all_verbose = False
    right_touch = 3
    left_touch = 2
    sonar = 1
    sonar_offset = 5
    # sonar rotation offset, 0 is facing forward, positive is left/anticlockwise
   

    #############################################################################
    ########     MAGIC METHODS    ###############################################
    #############################################################################

    def __init__(self, x, y, theta, draw=False, record=False):
        self.interface = brickpi.Interface()

        self.initialize()

        self.motorEnable(robot.wheel_motors[0])
        self.motorEnable(robot.wheel_motors[1])
        self.motorEnable(robot.sonar_motor)

        motorParams = self.interface.MotorAngleControllerParameters()
        motorParams.maxRotationAcceleration = 6.0
        motorParams.maxRotationSpeed = 12.0
        motorParams.feedForwardGain = 255 / 22.0
        motorParams.minPWM = 25
        motorParams.pidParameters.minOutput = -255
        motorParams.pidParameters.maxOutput = 255

        # proportional gain, reduces error
        motorParams.pidParameters.k_p = 270.0
        # integral gain, removes steady_state error
        motorParams.pidParameters.k_i = 400
        # differential gain, reduce settling time
        motorParams.pidParameters.k_d = 160

        self.setMotorAngleControllerParameters(robot.wheel_motors[0], motorParams)
        self.setMotorAngleControllerParameters(robot.wheel_motors[1], motorParams)
        self.setMotorAngleControllerParameters(robot.sonar_motor, motorParams)

        # initialise localisation
        self.loc = localisation(x,y,theta,draw, record)

        self.bumper_enabled = False
        self.in_recovery = False
         # sonar rotation offset, 0 is facing forward, positive is left/anticlockwise
        self.sonar_rotation_offset = 0 #initialise to 0

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

    @staticmethod
    def setAllVerbose(value):
        robot.all_verbose = value

    @staticmethod
    def getAllVerbose():
        return robot.all_verbose

    #############################################################################
    ########     PUBLIC MOVEMENT METHODS    #####################################
    #############################################################################
    #sonar spin
    def sonarTurnLeftDeg(self, degrees):
        self.turnSonar(math.radians(-degrees))
        self.sonar_rotation_offset+=degrees
        print "sonar_rotation_offset:"+str(self.sonar_rotation_offset)
    
    def sonarTurnRightDeg(self, degrees):
        self.turnSonar(math.radians(degrees))
        self.sonar_roation_offset-=degrees
        print "sonar_rotation_offset:"+str(self.sonar_rotation_offset)

    # distance in cm
    def forward(self, distance, verbose=False):
        self.linearMove(distance)
        self.loc.loc_distance(distance)
        if verbose or robot.all_verbose: print "Completed forward " + str(distance)

        self.loc.loc_distance(-distance)
        if verbose or robot.all_verbose: print "Completed backward " + str(distance)

    def turnRightRad(self, radius, verbose=False):
        length = radius * robot.wheel_separation / 2
        angle = length / robot.wheel_radius
        self.turn([angle, -angle])
        self.loc.loc_rotation(math.degrees(-radius))
        if verbose or robot.all_verbose: print "Completed right turn " + str(radius)

    def turnLeftRad(self, radius, verbose=False):
        length = radius * robot.wheel_separation / 2
        angle = length / robot.wheel_radius
        self.turn([-angle, angle])
        self.loc.loc_rotation(math.degrees(radius))
        if verbose or robot.all_verbose: print "Completed left turn " + str(radius)

    def turnRightDeg(self, degrees):
        self.turnRightRad(math.radians(degrees))

    def turnLeftDeg(self, degrees):
        self.turnLeftRad(math.radians(degrees))

    def turnRight90(self):
        self.turnRightRad(math.pi / 2)

    def turnLeft90(self):
        self.turnLeftRad(math.pi / 2)

    def turnDeg(self, degrees):
        if degrees < 180:
            self.turnLeftDeg(degrees)
        else:
            self.turnRightDeg(360 - degrees)

    def instantStop(self, verbose=False):
        self.setMotorPwm(0, 0)
        self.setMotorPwm(0, 0)
        if verbose or robot.all_verbose: print "Instant stop!!!"

    def navigateToWaypoint(self, x, y):
        currentX, currentY, theta = self.loc.get_average()
        dx = x - currentX 
        dy = y - currentY
        alpha = math.degrees(math.atan2(dy, dx))
        beta = robot.normalise_angle(alpha - theta)
        distance = math.hypot(dx, dy)
        self.turnDeg(beta)
        self.getSonarAndUpdate()
        currentX, currentY, theta = self.loc.get_average()
        dx = x - currentX 
        dy = y - currentY
        alpha = math.degrees(math.atan2(dy, dx))
        beta = robot.normalise_angle(alpha - theta)
        distance = math.hypot(dx, dy)

        if distance > 20:
            self.forward(20)
            self.getSonarAndUpdate()
            self.navigateToWaypoint(x,y)
        else:
            self.forward(distance)
            self.getSonarAndUpdate()
            print "-------------->IM AT THE WAYPOINT : " + str(x) + ", " + str(y)

    def rotateAndUpdate(self):
        currentX, currentY, theta = self.loc.get_average()
        # Turn to have theta 0 
        print "^~~~~~~^ I WANT TO ROTATEeeeee :3" 
        # Find out the closest angle in [0, 90, 180, 270]
        diffs = [theta-0, theta-90, theta-180, theta-270];
        
        self.turnDeg(90 - (theta - 0 ))
        self.getSonarAndUpdate()
        # Now the theta should be 90
        currentX, currentY, theta = self.loc.get_average()
        self.turnDeg(90 - (theta - 90))
        self.getSonarAndUpdate()
        # Now the theta should be 180
        currentX, currentY, theta = self.loc.get_average()
        self.turnDeg(90 - (theta - 180))
        self.getSonarAndUpdate()
        # Now the theta should be 270
        currentX, currentY, theta = self.loc.get_average()
        self.turnDeg(90 - (theta - 270))
        self.getSonarAndUpdate()


    def getSonarAndUpdate(self):
        # Get sonar measurements
        sonarMeasurements = self.getSonarMeasurements(200)
        # Update particles
        if len(sonarMeasurements) > 0:
            self.loc.update(sonarMeasurements)
        self.loc.drawAllParticles()

    #############################################################################
    ########     PUBLIC SENSOR METHODS    #######################################
    #############################################################################

    def enableBumper(self, verbose=False):
        self.sensorEnableTouch(robot.left_touch)
        self.sensorEnableTouch(robot.right_touch)
        if verbose or robot.all_verbose: print "Bumper Enabled"
        self.bumper_enabled = True

    def enableSonar(self, verbose=False):
        self.sensorEnableUltrasonic(robot.sonar)
        if verbose or robot.all_verbose: print "Sonar Enabled"

    def getSonarMeasurements(self, n):
        readings = []
        for i in range(n):
            reading = self.getSonarSingle() 
            if not reading == 255:
                readings.append(reading + robot.sonar_offset)
        return readings          
 
    def getSonarSingle(self):
        return self.getSensorValue(robot.sonar)[0]

    def disableBumper(self):
        self.sensorDisable(robot.left_touch)
        self.sensorDisable(robot.right_touch)
        self.bumper_enabled = False

    def disableSonar(self):
        self.sensorDisable(robot.sonar)

    def check_bumper(self):
        return self.getSensorValue(robot.left_touch)[0] or self.getSensorValue(robot.right_touch)[0]

    def recover(self):
        left = self.getSensorValue(robot.left_touch)[0]
        right = self.getSensorValue(robot.right_touch)[0]
        self.in_recovery = True
        self.loc.in_recovery = True
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
        self.loc.in_recovery = False
        self.rotateAndUpdate()
        self.in_recovery = False
        

    #############################################################################
    ########     LOCALISATION METHODS    ########################################
    #############################################################################

    def get_loc(self):
        return self.loc


    def findDistance(self, dist):
        # TODO: pick left/right at random
        self.turnSonarTillDistance(dist)
   
    def createSignature(self):
        measurements = self.turnSonarTakingMeasurements()
        # TODO: save?
        print measurements
        print len(measurements)


    #############################################################################
    ########     PRIVATE METHODS    #############################################
    #############################################################################

    @staticmethod
    def normalise_angle(angle):
        if angle < 0:
            return robot.normalise_angle(angle + 360)
        if angle > 360 :
            return robot.normalise_angle(angle - 360)
        else :
            return angle

    @staticmethod
    def wheelRadianTurn(radians):
        return (radians * robot.wheel_separation) / (2 * robot.wheel_radius)

    def turn(self, angles):
        self.increaseMotorAngleReferences(robot.wheel_motors, angles)
        while not self.motorAngleReferencesReached(robot.wheel_motors):
            time.sleep(0.1)

    def turnSonarTakingMeasurements(self):
        measurements = []
        step = 10
	for i in range(1,(360/step)+1):
            self.increaseMotorAngleReference(robot.sonar_motor, math.radians(step))
            while not self.motorAngleReferenceReached(robot.sonar_motor):
	        time.sleep(0.1)
            measurements.append(self.getSonarSingle())
        self.increaseMotorAngleReference(robot.sonar_motor, math.radians(-360))
	while not self.motorAngleReferenceReached(robot.sonar_motor):
        	time.sleep(0.1)  
	return measurements  
        
    def turnSonarTillDistance(self, distance):
        sonar = self.getSonarMeasurements(1)[0]
        sonars = [sonar]
        self.increaseMotorAngleReferences(robot.wheel_motors, [math.radians(360),math.radians(-360)])
        while not sonar in range(distance,distance):
            sonar = self.getSonarMeasurements(1)[0]
            sonars.append(sonar)
        print sonars
        self.instantStop(0)
        self.instantStop(1)
        # TODO: get new angle ref
        # TODO: turn back
        return -1

    def turnSonar(self, angle):
        self.increaseMotorAngleReference(robot.sonar_motor, angle)
        while not self.motorAngleReferenceReached(robot.sonar_motor):
            time.sleep(0.1)    

    # direction is true if forward, false if backward
    def linearMove(self, distance):
        angle = distance / robot.wheel_radius
        self.increaseMotorAngleReferences(robot.wheel_motors, [angle, angle])
        while not self.motorAngleReferencesReached(robot.wheel_motors):
            if self.bumper_enabled and self.check_bumper() and not self.in_recovery:
                self.instantStop()
                self.recover()
                break
            time.sleep(0.1)
import math
from localisation import localisation
