import random
import math
import time

NUM_OF_PARTS = 100

X = 0
Y = 1
THETA = 2

# x y theta
pos = [[0,0,0]]

# weightings will need doing

particles = pos * NUM_OF_PARTS

def print_format(particle_id):
	return (particles[particle_id][X], particles[particle_id][Y], particles[particle_id][THETA])

def ran_gauss(mu, sigma):
	return random.gauss(mu, sigma)

def update_particle_distance(particle_id, distance):
	e = 0.1
	f = 0.1
	particles[particle_id][X] = particles[particle_id][X] + (distance+e)*math.cos(particles[particle_id][THETA])
	particles[particle_id][Y] = particles[particle_id][Y] + (distance+e)*math.sin(particles[particle_id][THETA])
	particles[particle_id][THETA] = particles[particle_id][THETA] + f

def update_particle_rotation(particle_id, angle):
	alpha = 0
	g = 0
	particles[particle_id][THETA] = particles[particle_id][THETA] + alpha + g

for i in range(4):
	distance = 10
	for p in range(100):
		update_particle_distance(p, distance)
		print "drawParticles:" + str(print_format(p))
	time.sleep(1)
		
