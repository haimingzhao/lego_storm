import sys
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)


robot.sensorEnableUltrasonic(robot.sonar)
"""
m = robot.getSonarMeasurements(10)
print m
"""
robot.findDistance(63)
