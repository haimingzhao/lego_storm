import sys
import time
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)



#robot.findDistance(63)
while True:
    robot.sonarSpin(65)
    robot.followWallLeft(200,23)

    time.sleep(2)
    robot.sonarSpin(50)
    robot.followWallBackwards(200,23)
    robot.sonarReset()
    time.sleep(2)
