from robot import robot
import time

robot = robot()

speed = 100

robot.setMotorRotationSpeedReferences([0,1], [speed, speed])

print "Press Ctrl+C to exit"
while True:
	time.sleep(1)
	

robot.terminate()
