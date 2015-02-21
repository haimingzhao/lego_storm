
WALLS = [(0,0,0,168), (0,168,84,168), (84,126,84,210),
	 (84,210,168,210), (168,210,168,84), (168,84,210,84),
	 (210,84,210,0), (210,0,0,0)]


class WallMap:
    def __init__(self, loc):
        self.loc = loc
        self.walls = []
        self.walls += WALLS

    def isOnWall(self, point, wall):
        pass
	
    def clear(self):
        self.walls = []

    def draw(self):
        for a in self.walls:
            self.loc.draw_line(a[0], a[1], a[2], a[3])

    def get_walls(self):
        return self.walls
