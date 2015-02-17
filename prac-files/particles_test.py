import sys
sys.path.insert(0, '../')
from robot import robot
import time


robot = robot(True, True)
   
for a in range(4):
    for b in range(4):
        robot.forward(10)
    robot.turnRightDeg(90)
#draw_all()
    
    
    
