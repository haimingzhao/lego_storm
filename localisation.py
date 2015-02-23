import random
import copy
import math
import numpy as np
from wallMap import WallMap

class localisation:

    # Offsets for visualisation
    origin_offset_x = 200
    origin_offset_y = 200
    scalar = 10

    NUM_OF_PARTS = 100

    # Positions in particle vector
    X = 0
    Y = 1
    THETA = 2

    # value in cm
    LINEAR_DISTANCE = 0.39
    # value in degrees
    LINEAR_ROTATION = 0.1
    # value in degrees
    ROTATION = 0.39

    origin = [(0,0,0)]

    draw = False

    @staticmethod
    def norm_output(value):
        return round(2, value)

    @staticmethod
    def draw_particles(particles):
        p = copy.deepcopy(particles)
        for a in range(len(particles)):
            x = (p[a][localisation.X] * localisation.scalar) + localisation.origin_offset_x
            y = -(p[a][localisation.Y] * localisation.scalar) + localisation.origin_offset_y
            theta = p[a][localisation.THETA]
            p[a] = (x,y,theta)
        print "drawParticles:" + str(p)

    def __init__(self, drawing=False, record=False):
        self.particles = localisation.origin * localisation.NUM_OF_PARTS
        self.wallMap = WallMap(self)
        weight = float(1 / float(localisation.NUM_OF_PARTS))
        self.weightings = [weight] * localisation.NUM_OF_PARTS
        self.cumulative_weight = np.cumsum(self.weightings)
        global draw
        draw = drawing
        if record:
            self.record_all = []
            self.record_all += self.particles
            self.record = True
        else:
            self.record = False
        if drawing:
            localisation.draw_particles(self.particles)


    def wrap(self, angle):
        if angle < 0:
            return self.wrap(angle + 360)
        if angle > 360:
            return self.wrap(angle - 360)
        return angle

    def draw_line(self, x1, y1, x2, y2):
        x1 = (x1*self.scalar)+self.origin_offset_x
        y1 = (y1*self.scalar)+self.origin_offset_y
        x2 = (x2*self.scalar)+self.origin_offset_x
        y2 =(y2*self.scalar)+self.origin_offset_y
        line = (x1, y1, x2, y2)
        print "drawLine:" + str(line)

    @staticmethod
    def ran_gauss(sigma):
        return random.gauss(0, sigma)

    def update_particle_distance(self, pid, distance):
        e = localisation.ran_gauss(localisation.LINEAR_DISTANCE)
        f = localisation.ran_gauss(localisation.LINEAR_ROTATION)
        x = self.particles[pid][localisation.X] + (distance+e)*math.cos(math.radians(self.particles[pid][localisation.THETA]))
        y = self.particles[pid][localisation.Y] + (distance+e)*math.sin(math.radians(self.particles[pid][localisation.THETA]))
        theta = self.particles[pid][localisation.THETA] + f
        self.particles[pid] = (x,y,theta)


    def update_particle_rotation(self, pid, angle):
        g = localisation.ran_gauss(localisation.ROTATION)
        x = self.particles[pid][localisation.X]
        y = self.particles[pid][localisation.Y]
        theta = self.particles[pid][localisation.THETA] + angle + g
        self.particles[pid] = (x,y,theta)

    def loc_distance(self, d):
        if self.record: self.record_all += self.particles
        for p in range(localisation.NUM_OF_PARTS):
            self.update_particle_distance(p, d)
        if draw: self.draw_particles(self.particles)

    def loc_rotation(self, angle):
        if self.record: self.record_all += self.particles
        for p in range(localisation.NUM_OF_PARTS):
            self.update_particle_rotation(p, angle)
        if draw: self.draw_particles(self.particles)

    def draw_all(self):
        self.draw_particles(self.record_all)

    def drawAllParticles(self):
        self.draw_particles(self.particles)

    def get_average(self):
        av_x = 0
        av_y = 0
        av_t = 0
        for p in range(localisation.NUM_OF_PARTS):
            av_x += self.weightings[p] * self.particles[p][localisation.X]
            av_y += self.weightings[p] * self.particles[p][localisation.Y]
            av_t += self.weightings[p] * self.particles[p][localisation.THETA]
        return localisation.norm_output(av_x), localisation.norm_output(av_y), av_t

    # Updates weights of all the particles using the likelihood function
    def update(self, sonarMeasurements):
        z = np.median(sonarMeasurements)
        var = np.var(sonarMeasurements)
        total_weight = 0
        for i in range(localisation.NUM_OF_PARTS):
            particle = self.particles[i]
            likely = self.calculateLikelihood(particle, z, var)
            w = self.weightings[i]
            # Update the weighting of the particle based on likelyhood
            self.weightings[i] = likely * w
            total_weight += self.weightings[i]

        # Normalise the weights
        for i in range(localisation.NUM_OF_PARTS):
            self.weightings[i] = self.weightings[i] / total_weight

        # Sets the cumulative weight for resampling
        self.cumulative_weight = np.cumsum(self.weightings)

        # Resample
	particles_old = self.particles[:]
        for i in range(localisation.NUM_OF_PARTS):
            rand = np.random.random_sample()
            for j in range(localisation.NUM_OF_PARTS):
                if rand < self.cumulative_weight[j]:
                    self.particles[i]  = particles_old[j]
                    break


    # Returns a likelihood given a particle and mean sonar measurement
    def calculateLikelihood(self, p, z, var):
        m = self.getDepthMeasurement(p)
        diff = z - m
        top = -(pow(diff,2))
        bottom = 2 * var
        # Can add constant value K to make it more robust
        return math.e * (top / float(bottom))



    # Finds out which wall is the robot facing and gets "true" distance from it
    def getDepthMeasurement(self, p):
        # Represents the distance from each wall [(wall, distance)]
        wallDistances = []
        # Calculate the distance from each wall
        walls = self.wallMap.get_walls()
        for wall in walls:
            distance = self.calcDistanceFromWall(wall, p)
            if distance >0:
                wallDistances.append((wall, distance))
        # Sort the distances
        wallDistances = sorted(wallDistances, key=lambda x: x[1])

        # Take the smallest distance, while checking if the point actually is on the wall
        x,y,theta = p
        for wall, m in wallDistances:
            meetPoint = (x+m*math.cos(math.radians(theta)) , y+m*math.sin(math.radians(theta)))
            if self.wallMap.isOnWall(meetPoint, wall) and self.wallMap.reasonableAngle(theta, wall):
                print wall
                print meetPoint
                return m
        # Failed to find a distance
        return -1


    def calcDistanceFromWall(self, wall, p):
        Ax,Ay,Bx,By = wall
        x,y,theta = p
        cosTheta = math.cos(math.radians(theta))
        sinTheta = math.sin(math.radians(theta))
        top = (By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y)
        bottom = (By - Ay)*cosTheta - (Bx - Ax)*sinTheta
        return -1 if bottom == 0 else top / float(bottom)


    def setParticlesTo(self, position):
        for p in self.particles:
            p = position


    def getCumWeight(self):
        return self.cumulative_weight
