import sys
import time
sys.path.insert(0, '../')
from robot import robot
from place import place

##########   GLOBALS   ##################
half = 240
full = 480
small = 30
diswall = 24 # distance to look at wall
disout = 42 # distance to get out from trap

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)


#########---- ROBOT GO GO GO ---##########
#### ***Get out
measurements = robot.turnSonarTakingMeasurements()
print measurements
place.get_Loc(measurements)

robot.sonarSpin(-90)
toturn = robot.getMeanAngle(measurements)

robot.turnLeftDeg(toturn-13)
robot.forward(42)


#### ***Decide location
sonarRight = robot.getSonarMeasurements(1)[0]
robot.sonarSpin(-180)

sonarLeft = robot.getSonarMeasurements(1)[0]

position = (0,0,0)
if sonarRight > 30:
    if sonarLeft > 30:
        position = "middle"
    else:
        position = "left"
else:
    position = "right"


#### *** Go to other points
#robot.winTheChallenge(position)      
print "MY POSITION = " + str(position)
routine2(robot)
###################################################
#
########    METHODS TO WIN THE CHALLENGE    #######

def routineMid(robot):
    #robot.turnRightDeg(90)#TODO get rid of

    # AT 2: robot already looking at left wall
    robot.sonarSpin(-25)   # look forward to the left
    robot.followWallLeft(half, diswall)
    robot.turnRightDeg(90) # look at small wall
    robot.followWallLeft(small,diswall)
    print "!! I AM AT POINT 1 !!"

    # AT 1: robot is looking at leftwall forward 25 degree from staight
    robot.sonarSpin(50)   # look at backward left 25 degree
    robot.followWallBackwards(small,diswall)
    robot.turnLeftDeg(90) # look at long wall 
    robot.followWallBackwards(full, diswall)
    self.turnLeftDeg(90)
    self.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

    # AT 3: robot is looking at left wall  
    robot.sonarSpin(-50)
    robot.followWallLeft(small,21)
    robot.turnRightDeg(90)
    robot.followWallLeft(half,21)
    robot.turnRightDeg(90)
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

def routineLeft(robot):
    # AT 3: robot already looking at left wall
    robot.sonarSpin(-25)   # look forward to the left
    robot.followWallLeft(half, diswall)
    robot.turnRightDeg(90) # look at small wall
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

    # AT 2: robot is looking at leftwall forward 25 degree from staight
    robot.forward(-42) # move out
    robot.turnLeftDeg(90) # look at long wall 
    robot.followWallLeft(half, diswall)
    self.turnRightDeg(90)
    self.followWallLeft(small, diswall)
    print "!! I AM AT POINT 1 !!"

    # AT 1: robot is looking at left wall  
    robot.sonarSpin(50)
    robot.followWallLeft(small,diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(full, diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

def routineRight(robot):
    # AT 1: robot already looking at left wall
    robot.sonarSpin(25)   # look forward to the left
    robot.followWallBackwards(half, diswall)
    robot.turnRightDeg(90) # look at small wall
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

    # AT 2: robot is looking at leftwall forward 25 degree from staight
    robot.forward(-42) # move out
    robot.turnLeftDeg(90) # look at long wall 
    robot.followWallBackwards(half, diswall)
    self.turnLeftDeg(90)
    self.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

    # AT 3: robot is looking at left wall  
    robot.sonarSpin(-50)
    robot.followWallLeft(small,diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(full, diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(small, diswall)
    print "!! I AM AT POINT 1 !!"

# self is robot
def winTheChallenge(robot, position):
    self.turnRightDeg(90)
    if position=="middle":
        routineMid(robot)
    if position=="left":
        routineLeft(robot)
    if position == "right":
        routineRight(robot)
    print "WINNERS FROM LEEDSGHLEY"
