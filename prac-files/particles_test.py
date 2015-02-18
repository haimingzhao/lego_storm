import sys
sys.path.insert(0, '../')
from robot import robot
import time


robot = robot(True, True)

l = robot.get_loc()

l.draw_line(400, 400, 800, 400)
l.draw_line(800, 400, 800, 800)
l.draw_line(800, 800, 400, 800)
l.draw_line(400, 800, 400, 400)
   
for a in range(4):
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.forward(10)
    robot.turnRightDeg(90)

l.draw_all()
    
    
    
