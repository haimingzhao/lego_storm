import random
import math
import time

NUM_OF_PARTS = 100

scalar = 15
distance = 10 * scalar
turn = math.pi/2

X = 0
Y = 1
THETA = 2

# x y theta
pos = [(50,50,0)]

global all_parts
all_parts = []

# weightings will need doing

particles = pos * NUM_OF_PARTS

def ran_gauss(sigma):
    return random.gauss(0, sigma)

def update_particle_distance(particle_id, distance):
    e = ran_gauss(1)
    f = ran_gauss(0.01)
    x = particles[particle_id][X] + (distance+e)*math.cos(particles[particle_id][THETA])
    y = particles[particle_id][Y] + (distance+e)*math.sin(particles[particle_id][THETA])
    theta = particles[particle_id][THETA] + f
    particles[particle_id] = (x, y ,theta)

def update_particle_rotation(particle_id, angle):
    g = ran_gauss(0.01)
    x = particles[particle_id][X]
    y = particles[particle_id][Y]
    theta = particles[particle_id][THETA] + angle + g
    particles[particle_id] = (x, y ,theta)
    
def draw_square():
    line1 = (50, 50, 650, 50) # top line
    line2 = (650, 50, 650, 650)  # right line
    line3 = (650, 650, 50, 650)  # bottom line
    line4 = (50, 650, 50, 50)  # left line
    print "drawLine:" + str(line1)
    print "drawLine:" + str(line2)
    print "drawLine:" + str(line3)
    print "drawLine:" + str(line4)
    
def draw_start_pos():
    global all_parts
    all_parts += particles
    print "drawParticles:" + str(particles)
    
def draw_distance(d):
    global all_parts
    all_parts += particles
    for p in range(100):
        update_particle_distance(p, d)
    print "drawParticles:" + str(particles)
    
def draw_rotation(angle):
    global all_parts
    all_parts += particles
    for p in range(100):
        update_particle_rotation(p, angle)
    print "drawParticles:" + str(particles)
    
def draw_all():
    print "drawParticles:" + str(all_parts)

    
    
######## START HERE ##########

draw_square()
time.sleep(1)
draw_start_pos()
time.sleep(1)
for a in range(4):
    for b in range(4):
        draw_distance(distance)
        time.sleep(1)
    draw_rotation(turn)
    time.sleep(1)
draw_all()
    
    
    