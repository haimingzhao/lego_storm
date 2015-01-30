import calibration
import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

calibration.calibrate(interface, motors)

speed=100

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

print "Press Ctrl+C to exit"
while True:
	time.sleep(1)
	

interface.terminate()
