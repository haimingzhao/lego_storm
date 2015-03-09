import sys
import time
sys.path.insert(0, '../')
from robot import robot

robot = robot(21, 21, 180, True, True)


robot.enableSonar()
robot.enableBumper()

robot.followWall(200, 15) 



"""
while True:
    robot.setMotorRotationSpeedReference(0, 9)
    robot.setMotorRotationSpeedReference(1, 8)
    time.sleep(0.3)
    robot.setMotorRotationSpeedReference(0, 7)
    robot.setMotorRotationSpeedReference(1, 10)
 


m = robot.getSonarMeasurements(10)
print m


robot.findDistance(63)
"""