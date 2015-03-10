import sys
import time
sys.path.insert(0, '../')
from robot import robot
from place import place

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)

#robot.findDistance(63)
measurements = robot.turnSonarTakingMeasurements()
print measurements
place.get_Loc(measurements)

robot.sonarSpin(-90)
toturn = robot.getMeanAngle(measurements)


robot.turnLeftDeg(toturn-13)
robot.forward(42)


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

robot.winTheChallenge(position)   
    
print "MY POSITION = " + str(position)


