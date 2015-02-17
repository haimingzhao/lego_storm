import time
import math
import random

class localisation:

	global NUM_OF_PARTS
	global X
	global Y
	global THETA
	global LINEAR_DISTANCE
	global LINEAR_ROTATION
	global ROTATION
	global origin
	global draw

	NUM_OF_PARTS = 100

	X = 0
	Y = 0
	THETA = 2

	# value in cm
	LINEAR_DISTANCE = 0.5
	# value in degrees
	LINEAR_ROTATION = 0.5
	# value in degrees
	ROTATION = 2

	origin = [(50,50,0)]

	draw = False

	def __init__(self, drawing=False, record=False):
		self.particles = origin * NUM_OF_PARTS
		global draw
		draw = drawing
		if record:
			self.record_all = self.particles
			self.record = True
		else:
			self.record = False

	def draw_particles(self, particles):
		print "drawParticles:" + str(particles)

	def draw_line(self, x1, y1, x2, y2):
		line = (x1, y1, x2, y2)
		print "drawLine:" + str(line)

	def ran_gauss(self, sigma):
		return random.gauss(0, sigma)

	def degreeToRad(self, degree):
		return degree * math.pi / 180

	def update_particle_distance(self, pid, distance):
		e = self.ran_gauss(LINEAR_DISTANCE)
		f = self.ran_gauss(LINEAR_ROTATION)
		x = self.particles[pid][X] + (distance*e)*math.cos(self.particles[pid][THETA])
		y = self.particles[pid][Y] + (distance*e)*math.sin(self.particles[pid][THETA])
		theta = self.particles[pid][THETA] + f
		self.particles[pid] = (x,y,theta)

	def update_particle_rotation(self, pid, angle):
		g = self.ran_gauss(self.degreeToRad(ROTATION))
		x = self.particles[pid][X]
		y = self.particles[pid][Y]
		theta = self.particles[pid][THETA] + angle + g
		self.particles[pid] = (x,y,theta)

	def loc_distance(self, d):
		if self.record: self.record_all += self.particles
		for p in range(NUM_OF_PARTS):
			self.update_particle_distance(p, d)
		if draw: draw_particles(self.particles)

	def loc_rotation(self, angle):
		if self.record: self.record_all += self.particles
		for p in range(NUM_OF_PARTS):
			self.update_particle_rotation(p, angle)
		if draw: draw_particles(self.particles)

	def draw_all(self):
		draw_particles(self.record_all)

