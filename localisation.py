import time
import math

class robot:

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

	def __init__(self, draw=False, record=False):
		self.particles = origin * NUM_OF_PARTS
		draw = draw
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

	def ran_gauss(sigma):
		return random.gauss(0, sigma)

	def degreeToRad(self, degree):
		return degree * math.pi / 180

	def update_particle_distance(pid, distance):
		e = ran_gauss(LINEAR_DISTANCE)
		f = ran_gauss(LINEAR_ROTATION)
		x = self.particles[pid][X] + (distance*e)*math.cos(self.particles[pid][THETA])
		y = self.particles[pid][Y] + (distance*e)*math.sin(self.particles[pid][THETA])
		theta = self.particles[pid][THETA] + f
		self.particles[pid] = (x,y,theta)

	def update_particle_rotation(pid, angle):
		g = ran_gauss(degreeToRad(ROTATION))
		x = self.particles[pid][X]
		y = self.particles[pid][Y]
		theta = self.particles[pid][THETA] + angle + g
		self.particles[pid] = (x,y,theta)

	def loc_distance(d):
		if self.record: self.record_all += self.particles
		for p in range(NUM_OF_PARTS):
			update_particle_distance(p, d)
		if draw: draw_particles(self.particles)

	def loc_rotation(angle):
		if self.record: self.record_all += self.particles
		for p in range(NUM_OF_PARTS):
			update_particle_rotation(p, angle)
		if draw: draw_particles(self.particles)

	def draw_all():
		draw_particles(self.record_all)


