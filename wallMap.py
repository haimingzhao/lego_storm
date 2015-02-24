import math

WALLS = [(0,0,0,168), (0,168,84,168), (84,126,84,210),
	 (84,210,168,210), (168,210,168,84), (168,84,210,84),
	 (210,84,210,0), (210,0,0,0)]

ROUTE = [(84,30,180,30),
	 (180,30,180,54),
	 (180,54,126,54),
	 (126,54,126,168),
	 (126,168,126,126),
	 (126,126,30,54),
	 (30,54,84,54),
	 (84,54,84,30)]


class WallMap:

    def __init__(self, loc):
        self.loc = loc
        self.walls = []
        self.walls += WALLS
        self.route = []
        self.route += ROUTE

    @staticmethod
    def isOnWall(point, wall):
        wallA = (wall[0],wall[1])
        wallB = (wall[2],wall[3])

        # Distance between point and starting points of the wall
        distWallA = WallMap.distanceBetweenPoints(point, wallA)
        distWallB = WallMap.distanceBetweenPoints(point, wallB)

        # Length of the wall
        wallLength = WallMap.distanceBetweenPoints(wallA, wallB)
        return distWallA + distWallB == wallLength


    @staticmethod
    def reasonableAngle(theta, wall):
        angle = WallMap.getAngleOnWall(theta, wall)
        desired_val = math.radians(25)
        return angle < desired_val

    @staticmethod
    def getAngleOnWall(theta, wall):
        theta = math.radians(theta)
        Ax, Ay, Bx, By = wall
        left = (Ay - By) * math.cos(theta)
        right = (Bx - Ax) * math.sin(theta)
        top = left + right
        bottom = math.sqrt(pow((Ay - By), 2) + pow((Bx - Ax), 2))
        angle = math.acos(top / bottom)
        return angle


    @staticmethod
    def distanceBetweenPoints(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.hypot(x2 - x1, y2 - y1)

    def clear(self):
        self.walls = []

    def draw_walls(self):
        for a in self.walls:
            self.loc.draw_line(a[0], a[1], a[2], a[3])

    def draw_route(self):
        for a in self.route:
            self.loc.draw_line(a[0], a[1], a[2], a[3])

    def get_walls(self):
        return self.walls
