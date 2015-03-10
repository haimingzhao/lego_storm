import sys
sys.path.insert(0, '../')
from robot import robot
#import place

##########   GLOBALS   ##################
half = 240
full = 480
small = 30
diswall = 24 # distance to look at wall
disout = 42 # distance to get out from trap

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)

measurements = robot.turnSonarTakingMeasurements()
#place.get_Loc(measurements)

robot.sonarSpin(-90)
toturn = robot.getMeanAngle(measurements)

robot.turnLeftDeg(toturn-13)

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
robot.winTheChallenge()

########    METHODS TO WIN THE CHALLENGE    #######

def routineMid():
    robot.turnRightDeg(90)
    robot.sonarSpin(-25)
    robot.followWallLeft(half, diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(small,diswall)
    print "!! I AM AT POINT 1 !!"

    robot.sonarSpin(50)
    robot.followWallBackwards(small,diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(full, diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

    robot.sonarSpin(-50)
    robot.followWallLeft(small,21)
    robot.turnRightDeg(90)
    robot.followWallLeft(half,21)
    robot.turnRightDeg(90)
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

def routineLeft():
    robot.turnRightDeg(90)
    robot.sonarSpin(-25)
    robot.followWallLeft(half, diswall)
    robot.turnRightDeg(90)
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

    robot.forward(-42)
    robot.turnLeftDeg(90)
    robot.followWallLeft(half, diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(small, diswall)
    print "!! I AM AT POINT 1 !!"

    robot.sonarSpin(50)
    robot.followWallBackwards(small,diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(full, diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

def routineRight():
    robot.turnRightDeg(90)
    robot.sonarSpin(-25)
    robot.followWallBackwards(half, diswall)
    robot.turnRightDeg(90)
    robot.forward(42)
    print "!! I AM AT POINT 2 !!"

    robot.forward(-42)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(half, diswall)
    robot.turnLeftDeg(90)
    robot.followWallBackwards(small, diswall)
    print "!! I AM AT POINT 3 !!"

    robot.sonarSpin(-50)
    robot.followWallLeft(small,diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(full, diswall)
    robot.turnRightDeg(90)
    robot.followWallLeft(small, diswall)
    print "!! I AM AT POINT 1 !!"

def winTheChallenge():
    robot.turnRightDeg(90)
    if position=="middle":
        routineMid(robot)
    if position=="left":
        routineLeft(robot)
    if position == "right":
        routineRight(robot)
    print "WINNERS FROM LEEDSGHLEY"
