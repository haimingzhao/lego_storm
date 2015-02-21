import sys 
from robot import robot
import time
import math
import copy

class mlc:

    INITIAL_X = 84
    INITIAL_Y = 20
    INITIAL_THETA = 0

    def __init__(self):
        pass

#This sets up for the waypoint test.
    def set_waypoint_start(particles):
        p = copy.deepcopy(particles)
        for i in range(len(particles)):
            p[i] = (mlc.INITIAL_X, mlc.INITIAL_Y, mlc.INITIAL_THETA)

        return p 

# This function will take all particles and update them.
    def update(self, robot):
        med_filter = []
        for i in range(100):
            #Make sure that the sensor is turned on.
            val = robot.getSensorValue(port)
            med_filter = med_filter.append(val) 

        #Using the median to smooth out garbage
        median = self.getMedian(med_filter)  
        l = robot.get_loc()
        p = l.get_particles()
        #for a in range(len(p)):
            #find difference and calculate new weight         

        return 0 

    def calculate_likelihood(x ,y ,theta ,z):
       return 0 



    def getMedian(numericValues):
        theValues = sorted(numericValues)
        if len(theValues) % 2 == 1:
            return theValues[(len(theValues)+1)/2-1]
        else:
            lower = theValues[len(theValues)/2-1]
            upper = theValues[len(theValues)/2]
            return (float(lower + upper)) / 2  
