import sys
sys.path.insert(0, '../')
from robot import robot
import time


robot = robot(True, True)

l = robot.get_loc()

l.draw_line(0, 0, 40, 0)
l.draw_line(40, 0, 40, 40)
l.draw_line(40, 40, 0, 40)
l.draw_line(0, 40, 0, 0)
   
for a in range(4):
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.turnRightDeg(90)

l.draw_all()

    
    
    
