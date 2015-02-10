from robot import robot
import math

#[x,y,theta]
startPos = [0,0,0]

NUMBER_OF_PARTICLES = 100

for i in range(4):
    robot.forward(10)
    #second argument is the standard deviation
    rand = random.gauss(mu, 0.1)
    startPos[0] += (10 + rand) * math.cos(startPos[2])
    time.sleep(3)

	

robot.terminate()
