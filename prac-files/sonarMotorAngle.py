import sys
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

m = robot.turnSonarTakingMeasurements()

print m
print len(m)

