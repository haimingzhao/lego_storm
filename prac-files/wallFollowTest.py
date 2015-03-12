import sys
import time
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)



#robot.findDistance(63)
while True:
    robot.sonarSpin(110)
    robot.followWallBackwards(400,25)
    time.sleep(2)
    robot.sonarSpin(-40)
    robot.followWallLeft(400,25)
    time.sleep(2)
    robot.sonar_reset();

