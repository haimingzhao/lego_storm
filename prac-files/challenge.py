import sys
sys.path.insert(0, '../')
from robot import robot
from datetime import datetime
# import place
import time

##########   GLOBALS   ##################
half = 256
half_fromwall = 273
full = half*2
small = 42
earlystop = 20
diswall = 21 # distance to look at wall
disout = 42 # distance to get out from trap
angle = 20





def runToWallAt21(forward=1): # -1 for backward
    toWall = robot.getSonarSingle() + 5 - 21
    if forward == -1:
        toWall -= 2
    print "still need = " + str(toWall)
    robot.forward(toWall*forward)

def routineMid():
    robot.sonarSpin(-angle)
    robot.followWallLeft(half-earlystop, diswall)
    robot.sonarSpin(-(90-angle)) # look at right wall 
    runToWallAt21()

    robot.turnRightDeg(90)
    distance = robot.getSonarSingle() - 21

    robot.sonarSpin((90-angle)) 

    robot.followWallLeft(distance,diswall)
    print "!! I AM AT POINT 1 !!"
    time.sleep(1)

    robot.sonarSpin(angle*2)
    robot.followWallBackwards(small,diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(full-earlystop, diswall)
    robot.sonarSpin((90-angle)) # look at right wall 
    runToWallAt21(-1)
   
    robot.turnLeftDeg(90)
    distance = robot.getSonarSingle() - 21 + 2 
    robot.sonarSpin(-(90-angle))

    robot.followWallBackwards(distance, diswall)
    print "!! I AM AT POINT 3 !!"
    time.sleep(1)

    robot.sonarSpin(-angle*2)
    robot.followWallLeft(small,diswall)
    robot.turnRightDeg(90)
    
    robot.followWallLeft(half/2,diswall)
    robot.sonarSpin(90+angle)
    distance = half_fromwall - robot.getSonarSingle()
    robot.sonarSpin(-(90+angle))
    robot.followWallLeft(distance, diswall)

    robot.turnRightDeg(90)

    robot.sonarSpin(-(90-angle)) # 
    runToWallAt21()

    print "!! I AM AT POINT 2 !!"

def routineLeft():
    robot.sonarSpin(-angle)
    robot.followWallLeft(half/2, diswall)
    robot.sonarSpin(90+angle)
    distance = half_fromwall - robot.getSonarSingle()
    robot.sonarSpin(-(90+angle))
    robot.followWallLeft(distance,diswall)
    robot.turnRightDeg(90)

    robot.sonarSpin(-(90-angle)) # 
    runToWallAt21()
    # robot.forward(42)
    print "!! I AM AT POINT 2 !!"
    time.sleep(1)

    robot.forward(-42)
    robot.turnLeftDeg(90)
    robot.sonarSpin(90-angle)

    robot.followWallLeft(half-earlystop, diswall)
    robot.sonarSpin(-(90-angle)) # look at right wall 
    runToWallAt21()
    robot.turnRightDeg(90)
    distance = robot.getSonarSingle() - 21
    robot.sonarSpin((90-angle)) #turn back for wall following
    
    robot.followWallLeft(distance, diswall)
    print "!! I AM AT POINT 1 !!"
    time.sleep(1)

    robot.sonarSpin(2*angle)
    robot.followWallBackwards(small,diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(full-earlystop, diswall)

    robot.sonarSpin((90-angle)) # look at right wall 
    runToWallAt21(-1)

    robot.turnLeftDeg(90)   
    distance = robot.getSonarSingle() - 21 + 2 
    robot.sonarSpin(-(90-angle)) #turn back for wall following

    robot.followWallBackwards(distance, diswall)
    print "!! I AM AT POINT 3 !!"

def routineRight():
    robot.sonarSpin(angle)
    robot.followWallBackwards(half/2, diswall)
    robot.sonarSpin(-(90+angle))
    distance = half_fromwall - robot.getSonarSingle()
    robot.sonarSpin(90+angle)
    robot.followWallBackwards(distance, diswall)
    robot.turnRightDeg(90)
    #turn sonar to wall and take measurement
    robot.sonarSpin(-(90+angle)) # i am looking at small wall
    runToWallAt21() 
    print "!! I AM AT POINT 2 !!"
    time.sleep(1)

    robot.forward(-42)
    robot.turnLeftDeg(90)
    robot.sonarSpin(90+angle)# turn back to look at left wall

    robot.followWallBackwards(half-earlystop, diswall)
    robot.sonarSpin(90-angle) # look at right wall 
    runToWallAt21(-1)

    robot.turnLeftDeg(90)
    distance = robot.getSonarSingle() - 21 + 2 
    robot.sonarSpin(-(90-angle)) #turn back for wall following

    robot.followWallBackwards(distance, diswall)
    print "!! I AM AT POINT 3 !!"
    time.sleep(1)

    robot.sonarSpin(-(angle*2))
    robot.followWallLeft(small,diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(full-earlystop, diswall)
    
    robot.sonarSpin(-(90-angle)) # look at right wall 
    runToWallAt21()

    robot.turnRightDeg(90)
    distance = robot.getSonarSingle() - 21
    robot.sonarSpin(90-angle) #turn back for wall following
    
    robot.followWallLeft(distance, diswall)
    print "!! I AM AT POINT 1 !!"

def winTheChallenge():
    robot.turnRightDeg(90)
    if position=="middle":
        routineMid()
    if position=="left":
        routineLeft()
    if position == "right":
        routineRight()
    print "WINNERS FROM LEEDSGHLEY"


robot = robot(0, 0, 0, True, True)
robot.enableSonar()


# Fast sensor motor params
fastParams = robot.interface.MotorAngleControllerParameters()
fastParams.maxRotationAcceleration = 7.0
fastParams.maxRotationSpeed = 12.0
fastParams.feedForwardGain = 255 / 15
fastParams.minPWM = 25
fastParams.pidParameters.minOutput = -255
fastParams.pidParameters.maxOutput = 255
fastParams.pidParameters.k_p = 270
fastParams.pidParameters.k_i = 400
fastParams.pidParameters.k_d = 160

# (273,21,-90), (525,21,-90), (21,21,-90)

startTime = datetime.now()
measurements = robot.turnSonarTakingMeasurements()

# Change sonar to move FASTAAAAAAAAAAAAR!
robot.setMotorAngleControllerParameters(robot.sonar_motor, fastParams)

# place.get_Loc(measurements)

robot.sonarSpin(-90)
toturn = robot.getMeanAngle(measurements) -13
toturn = toturn if toturn <= 180 else toturn - 360 

robot.turnLeftDeg(toturn)

robot.forward(disout)




#### ***Decide location
sonarRight = robot.getSonarMeasurements(1)[0]


robot.sonarSpin(-180)

sonarLeft = robot.getSonarMeasurements(1)[0]

if sonarRight > 30:
    if sonarLeft > 30:
        position = "middle"
    else:
        position = "left"
else:
    position = "right"


#### *** Go to other points
print "MY POSITION = " + position
winTheChallenge()
print datetime.now() - startTime
# print "REMEBER TO PUT 1 SECOND STOP AT EACH POINT"
