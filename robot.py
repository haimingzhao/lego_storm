import brickpi
from localisation import localisation
from wallMap import WallMap
import time
import math
import numpy as np

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
    sonar_offset = 0
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
        motorParams.maxRotationAcceleration = 3.0
        motorParams.maxRotationSpeed = 6.0
        motorParams.feedForwardGain = 255 / 30.0
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
        
        motorParams = self.interface.MotorAngleControllerParameters()
        motorParams.maxRotationAcceleration = 2.0
        motorParams.maxRotationSpeed = 3
        motorParams.feedForwardGain = 255 / 20.0
        motorParams.minPWM = 27
        motorParams.pidParameters.minOutput = -255
        motorParams.pidParameters.maxOutput = 255

        # proportional gain, reduces error
        motorParams.pidParameters.k_p = 270.0
        # integral gain, removes steady_state error
        motorParams.pidParameters.k_i = 200
        # differential gain, reduce settling time
        motorParams.pidParameters.k_d = 160
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
        self.setMotorPwm(1, 0)
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
        self.forward(-10)
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

    ##############SONAR TURNING#################
    #sonar spin left/anti-clockwise
    def sonarSpin(self, degrees):
        self.increaseMotorAngleReference(robot.sonar_motor, math.radians(-degrees))
        while not self.motorAngleReferenceReached(robot.sonar_motor):
            time.sleep(0.1)
        self.sonar_rotation_offset += degrees
        print "SONAR rotation offset : " + str(self.sonar_rotation_offset)
   
    #sonar return back to origin location
    def sonarReset(self):
        self.sonarSpin(-self.sonar_rotation_offset)
  
    def turnSonarTakingMeasurementsSteps(self):
        measurements = []
        step = 10
        for i in range(1,(360/step)+1):
            self.sonarSpin(step)
            measurements.append(self.getSonarSingle())
        return measurements  

    def turnSonarTakingMeasurements(self):
        # sonar = self.getSonarMeasurements(1)[0]
        measurements = []
        initialMotorAngle = self.getMotorAngle(robot.sonar_motor)[0]
        step = 2*math.pi / 360.0
        next_step = initialMotorAngle 
        # left turning
        step_measurements = []
        self.increaseMotorAngleReference(robot.sonar_motor, -math.radians(360))
        while not self.motorAngleReferenceReached(robot.sonar_motor):
            motorAngle = self.getMotorAngle(robot.sonar_motor)[0]
            step_measurements.append(self.getSonarMeasurements(1)[0])
            if motorAngle < next_step and len(measurements)<360:
                measurements.append(int(np.median(step_measurements)))
                step_measurements = []
                next_step -= step
        
        self.sonar_rotation_offset += 360
        return measurements

    @staticmethod
    def getMeanAngle(sonarMeasurements):
        top = 100
        low = 55
        ranges = []
        top_i = 0
        low_i = 0 
        j = 0
        started = False
        for m in sonarMeasurements:
            if m in range(low,top):
                if started:
                    top_i += 1
                else:
                    started = True
                    low_i = j
                    top_i = j+1
            else:
                if started:
                    ranges.append((low_i,top_i))
                    started = False
            j += 1
        if started:
            ranges.append((low_i,top_i))
        assert(len(ranges)<=2)
        print ranges
        mid_angle = 0
        if len(ranges)==1:
            mid_angle = (ranges[0][0] + ranges[0][1] ) / 2.0
            pass
        if len(ranges)==2:
            wrap_diff = ranges[1][0] - len(sonarMeasurements)
            print wrap_diff
            mid_angle = (ranges[0][1] + wrap_diff) / 2.0
            print mid_angle
 
        return mid_angle
        
    def turnSonarTillDistance(self, distance):
        length = 2*math.pi * robot.wheel_separation / 2
        angle = length / robot.wheel_radius
        sonar = self.getSonarMeasurements(1)[0]
        sonars = [sonar]
        self.increaseMotorAngleReferences(robot.wheel_motors, [angle,-angle])
        while not (sonar in range(distance-1,distance+1)):
            sonar = self.getSonarMeasurements(1)[0]
            sonars.append(sonar)

        self.instantStop(0)
        self.instantStop(1)
        print sonars
        return -1
        

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

    def followWallLeft(self, distance, wallDistance):
        vc = 8 # TODO
        Kp = 0.6 # TODO
        maxV = 16
        initial0,initial1 = self.getMotorAngles(self.wheel_motors)
        angle_toreach0 = (distance / robot.wheel_radius) + initial0[0]
        angle_toreach1 = (distance / robot.wheel_radius) + initial1[0]

        while (self.getMotorAngle(0)[0]<angle_toreach0 and self.getMotorAngle(1)[0]<angle_toreach1):
            sonar = self.getSonarMeasurements(1)[0] - self.sonar_offset - 1
            diff = sonar - wallDistance            
            if sonar < 45 and sonar > 10:
                #print "diff = " + str(diff)
                #print "sonar = " + str(sonar)
                vl = min(vc - 0.5*Kp*diff, maxV)
                vr = min(vc + 0.5*Kp*diff, maxV)
                vl = max(vl, 1)
                vr = max(vr, 1)       
                self.setMotorRotationSpeedReferences([0,1], [vl,vr])
                #print "new speed left = " + str(vl)
                #print "new speed right = " + str(vr)
        self.instantStop()



    def followWallBackwards(self, distance, wallDistance):
        vc = 8 # TODO
        Kp = 0.6 # TODO
        maxV = 16
        initial0,initial1 = self.getMotorAngles(self.wheel_motors)
        angle_toreach0 = initial0[0] - (distance / robot.wheel_radius) 
        angle_toreach1 = initial1[0] - (distance / robot.wheel_radius) 

        while (self.getMotorAngle(0)[0]>angle_toreach0 and self.getMotorAngle(1)[0]>angle_toreach1):
            sonar = self.getSonarMeasurements(1)[0] - self.sonar_offset - 1
            diff = sonar - wallDistance           
            if sonar < 45 and sonar > 10:
                #print "diff = " + str(diff)
                #print "sonar = " + str(sonar)
                vl = min(vc - 0.5*Kp*diff, maxV)
                vr = min(vc + 0.5*Kp*diff, maxV)
                vl = max(vl, 1)
                vr = max(vr, 1)       
                self.setMotorRotationSpeedReferences([0,1], [-vl,-vr])
                #print "new speed left = " + str(vl)
                #print "new speed right = " + str(vr)
        self.instantStop()
        """
        self.setMotorRotationSpeedReferences([0,1], [0,0])
        while not (self.motorRotationSpeedReferenceReached(0) and self.motorRotationSpeedReferenceReached(1)):
                    time.sleep(0.1)
        """
    def winTheChallenge(self, position):
       half = 240
       full = 480
       small = 30
       if position=="middle":
           self.turnRightDeg(90)
           self.followWallLeft(half,21)#251
           self.turnRightDeg(90)
           self.followWallLeft(small,21)#42
           # WE SHOULD BE AT POINT 1 NOW 
           self.followWallBackwards(small,21)#42
           self.turnLeftDeg(90)
           self.followWallBackwards(full,21)#504
           self.turnLeftDeg(90)
           self.followWallBackwards(small,21)#42
           # WE SHOULD BE AT POINT 3 NOW 
           self.followWallLeft(small,21)
           self.turnRightDeg(90)
           self.followWallLeft(half,21)
           self.turnRightDeg(90)
           self.forward(42)
       if position=="left":
           self.turnRightDeg(90)
           self.followWallLeft(half,21)
           self.turnRightDeg(90)
           self.forward(42)
           self.forward(-42)
           self.turnLeftDeg(90)
           self.followWallLeft(half,21)
           self.turnRightDeg(90)
           self.followWallLeft(small,21)
           self.followWallBackwards(small,21)
           self.turnLeftDeg(90)
           self.followWallBackwards(full,21)
           self.turnLeftDeg(90)
           self.followWallBackwards(small,21)
       if position == "right":
           self.turnRightDeg(90)
           self.followWallBackwards(half,21)
           self.turnLeftDeg(90)
           self.forward(-42)
           self.forward(42)
           self.turnRightDeg(90)
           self.followWallBackwards(half,21)
           self.turnLeftDeg(90)
           self.followWallBackwards(small,21)
           self.followWallLeft(small,21)
           self.turnRightDeg(90)
           self.followWallLeft(full,21)
           self.turnRightDeg(90)
           self.followWallLeft(small,21)
       print "WINNERS FROM LEEDSGHLEY"


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
