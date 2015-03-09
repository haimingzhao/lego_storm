import sys
import time
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)
robot.enableSonar()


#robot.findDistance(63)
measurements = robot.turnSonarTakingMeasurementsSteps()
print measurements
toturn = robot.getMeanAngle(measurements)*10
robot.turnLeftDeg(toturn)
robot.forward(42)
"""
robot.forward(42)
robot.turnRightDeg(90)
robot.sonarSpin(90)
robot.followWall(100, 21) 
robot.sonarReset()
"""
