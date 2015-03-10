from robot import robot
import sys

robot = robot()

try:
	from msvcrt import getch
except ImportError:
	def getch():
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

motors = [0,1]
speed = 150

def backward():
	robot.instantStop()
	robot.setMotorRotationSpeedReferences(motors, [-speed, -speed])

def forward():
	robot.instantStop()
	robot.setMotorRotationSpeedReferences(motors, [speed,speed])

def stop():
	robot.instantStop()

def right():
	robot.instantStop()
	robot.setMotorRotationSpeedReferences(motors, [speed, 0])

def left():
	robot.instantStop()
	robot.setMotorRotationSpeedReferences(motors, [0, speed])

stopped = True
char = getch()
while True:
	if char == 'q':
		break
	elif char == 'w':
		print "Going Forward"
		stopped = False
		forward()
	elif char == 's':
		if stopped:
			print "Going Backwards"
			stopped = False
			backward()
		else:
			print "Stopped"
			stopped = True
			stop()
	elif char == 'd':
		print "Going Right"
		stopped = False
		right()
	elif char == 'a':
		print "Going Left"
		stopped = False
		left()
	else:
		print "Unknown"
	char = getch()

robot.terminate()
