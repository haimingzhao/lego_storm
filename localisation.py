import time
import math
import random
import copy

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
	Y = 1
	THETA = 2

	# value in cm
	LINEAR_DISTANCE = 0.5
	# value in degrees
	LINEAR_ROTATION = 0.5
	# value in degrees
	ROTATION = 2

	origin = [(0,0,0)]

	draw = False

	def __init__(self, drawing=False, record=False):
		self.particles = origin * NUM_OF_PARTS
		self.weightings = [1/NUM_OF_PARTS] * NUM_OF_PARTS
		global draw
		draw = drawing
		if record:
			self.record_all = copy.deepcopy(self.particles)
			self.record = True
		else:
			self.record = False
		if drawing:
			self.draw_particles(self.particles)

	def wrap(self, angle):
		if angle < 0:
			angle += 360
		if angle > 360:
			angle -= 360
		return angle

	def draw_particles(self, particles):
		p = copy.deepcopy(self.particles)
		scalar = 10
		for a in range(NUM_OF_PARTS):
			x = (p[a][X] * scalar) + 100
			y = (p[a][Y] * scalar) + 100
			theta = p[a][THETA]
			p[a] = (x,y,theta)
			#print p[a]
		print "drawParticles:" + str(p)

	def draw_line(self, x1, y1, x2, y2):
		line = (x1, y1, x2, y2)
		print "drawLine:" + str(line)

	def ran_gauss(self, sigma):
		return random.gauss(0, sigma)

	def update_particle_distance(self, pid, distance):
		e = self.ran_gauss(LINEAR_DISTANCE)
		f = self.ran_gauss(LINEAR_ROTATION)
		x = self.particles[pid][X] + (distance+e)*math.cos(math.radians(self.particles[pid][THETA]))
		y = self.particles[pid][Y] + (distance+e)*math.sin(math.radians(self.particles[pid][THETA]))
		theta = self.particles[pid][THETA] + f
		self.wrap(theta)
		self.particles[pid] = (x,y,theta)

	def update_particle_rotation(self, pid, angle):
		g = self.ran_gauss(ROTATION)
		x = self.particles[pid][X]
		y = self.particles[pid][Y]
		theta = self.particles[pid][THETA] + angle + g
		self.wrap(theta)
		self.particles[pid] = (x,y,theta)

	def loc_distance(self, d):
		if self.record:
			self.record_all.extend(self.particles)
		for p in range(NUM_OF_PARTS):
			self.update_particle_distance(p, d)
		if draw: self.draw_particles(self.particles)

	def loc_rotation(self, angle):
		if self.record:
			self.record_all.extend(self.particles)
		for p in range(NUM_OF_PARTS):
			self.update_particle_rotation(p, angle)
		if draw: self.draw_particles(self.particles)

	def draw_all(self):
		self.draw_particles(self.record_all)

	def get_average(self):
		av_x = 0
		av_y = 0
		av_t = 0
		for p in range(NUM_OF_PARTS):
			av_x += self.weightings[p] * self.particles[p][X]
			av_y += self.weightings[p] * self.particles[p][Y]
			av_t += self.weightings[p] * self.particles[p][THETA]
		return [av_x, av_y, av_t]



