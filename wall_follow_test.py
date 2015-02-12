from robot import robot
import time

robot = robot()

robot.enableSonar()

robot.setWallFollowDistance(30)

robot.wallFollow()
