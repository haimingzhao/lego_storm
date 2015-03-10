import sys
sys.path.insert(0, '../')
from robot import robot

port = 1 # port which ultrasoic sensor is plugged in to
positions = [(180, 30),
             (180, 54),
             (126, 54),
             (126, 168),
             (126, 126),
             (30, 54),
             (84, 54),
             (84, 30)]

robot = robot(84, 30, 0, True, True)
robot.sensorEnableUltrasonic(port)

# remember to change origin in localisation to (84,30)

for p in positions:
    x, y = p
    robot.navigateToWaypoint(x, y)
    # if p in ((126, 126), (30, 54)) :
    #    robot.rotateAndUpdate()
    #    robot.navigateToWaypoint(x, y)
    
robot.get_loc().draw_all()

robot.terminate()
