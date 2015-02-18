import sys
sys.path.insert(0, '../')
from robot import robot
import time


robot = robot(True, True)

l = robot.get_loc()

l.draw_line(500, 500, 900, 500)
l.draw_line(900, 500, 900, 100)
l.draw_line(900, 100, 500, 100)
l.draw_line(500, 100, 500, 500)
   
for a in range(4):
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.turnRightDeg(90)

l.draw_all()
    
    
    
