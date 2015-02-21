import math

WALLS = [(0,0,0,168), (0,168,84,168), (84,126,84,210),
	 (84,210,168,210), (168,210,168,84), (168,84,210,84),
	 (210,84,210,0), (210,0,0,0)]


class WallMap:
    def __init__(self, loc):
        self.loc = loc
        self.walls = []
        self.walls += WALLS

    def isOnWall(self, point, wall):
	wallA = (wall[0],wall[1])
	wallB = (wall[2],wall[3])	

        # Distance between point and starting points of the wall
        distWallA = self.distanceBetweenPoints(point, wallA)
        distWallB = self.distanceBetweenPoints(point, wallB)

        # Length of the wall
        wallLength = self.distanceBetweenPoints(wallA, wallB)
	return distWallA + distWallB == wallLength	

    def distanceBetweenPoints(self, p1, p2):
	x1,y1 = p1
	x2,y2 = p2
	return math.hypot(x2 - x1, y2 - y1) 

    def clear(self):
        self.walls = []

    def draw(self):
        for a in self.walls:
            self.loc.draw_line(a[0], a[1], a[2], a[3])

    def get_walls(self):
        return self.walls
