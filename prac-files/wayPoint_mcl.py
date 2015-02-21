import sys
sys.path.insert(0, '../')
from robot import robot

positions = [(180, 30),
             (180, 54),
             (126, 54),
             (126, 168),
             (126, 126),
             (30, 54),
             (84, 54),
             (84, 30)]

robot = robot()

robot.enableBumper()

# remember to change origin in localisation to (84,30)

for p in positions:
    x, y = p
    robot.navigateToWaypoint(x, y)

robot.terminate()