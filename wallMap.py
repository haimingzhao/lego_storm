
WALLS = [(0,0,0,168), (0,168,84,168), (84,126,84,210),
	 (84,210,168,210), (168,210,168,84), (168,84,210,84),
	 (210,84,210,0), (210,0,0,0)]


class WallMap:
    def __init__(self):
	self.walls = []
	for wall in WALLS:
	    self.add_wall(wall)

    def add_wall(self, wall):
	self.walls.append(wall)

    def clear(self):
	self.walls = []

    # TODO: implement this
    def draw(self):
	pass

    def get_walls(self):
	return self.walls
