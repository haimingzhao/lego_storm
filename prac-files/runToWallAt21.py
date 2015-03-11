import sys
import time
sys.path.insert(0, '../')
from robot import robot
robot = robot(0, 0, 0, True, True)
robot.enableSonar()

def runToWallAt21(forward=1): # -1 for backward
    toWall = robot.getSonarSingle() + 5 - 21
    if forward == -1:
        toWall -= 2
    print "still need = " + str(toWall)
    robot.forward(toWall*forward)


# (273,21,-90), (525,21,-90), (21,21,-90)
runToWallAt21(-1)




