import sys
sys.path.insert(0, '../')
from localisation import localisation
import time
import math

loc = localisation(True, True)

for a in range(4):
	for b in range(4):
		print "Move forward 10"
		loc.loc_distance(10)
		time.sleep(1)
	print "Turn right 90"
	loc.loc_rotation(90)
	time.sleep(1)

#loc.draw_all()
