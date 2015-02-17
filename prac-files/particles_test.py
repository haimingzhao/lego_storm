import sys
sys.path.insert(0, '../')
from robot import robot
import time


robot = robot(True, True)
   
for a in range(4):
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.turnRightDeg(90)

robot.get_loc().draw_all()
    
    
    
