import sys
import time
sys.path.insert(0, '../')
from robot import robot

robot = robot(0, 0, 0, True, True)
robot.enableSonar()

# (273,21,-90), (525,21,-90), (21,21,-90)



#robot.findDistance(63)
while True:
    robot.sonarSpin(115)
    robot.followWallBackwards(200,25)
    time.sleep(2)
    robot.sonarSpin(-50)
    robot.followWallLeft(200,25)
    time.sleep(2)
    robot.sonar_reset();


