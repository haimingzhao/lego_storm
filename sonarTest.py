from robot import robot
r = robot(0,0,0)
r.enableSonar()
while True:
        print r.getSonarMeasurements(30)
