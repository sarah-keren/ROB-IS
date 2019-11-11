#!/usr/bin/env python
import rospy
import math  
from math import floor
from itertools import product
from nav_msgs.msg import OccupancyGrid


class HiddenCostMap:
    def __init__(self):
        self._static_map = None
        self._hidden_costmap = None
        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)

        if rospy.has_param('~peaks'):
            self.peaks = rospy.get_param('~peaks')
        if rospy.has_param('~doughnuts'):
            self.doughnuts = rospy.get_param('~doughnuts')
        if rospy.has_param('~bananas'):
            self.bananas = rospy.get_param('~bananas')

        rospy.Subscriber("map", OccupancyGrid, self.set_map)

    def set_map(self, msg):
        self._static_map = msg

    def _static_map_cell(self, x, y):
        gx = int((x - self._static_map.info.origin.position.x)/self._static_map.info.resolution)
        gy = int((y - self._static_map.info.origin.position.y)/self._static_map.info.resolution)
        return self._static_map.data[gx + gy*self._static_map.info.width]

    def _sign(self, n):
        return (n > 0) - (n < 0)

    # return if there is a collision between points p and c
    def _checkCollision(self, p, c):

        (xA, yA) = (p[0], p[1])
        (xB, yB) = (c[0], c[1])
        (dx, dy) = (xB - xA, yB - yA)
        sx = self._static_map.info.resolution * dx/(abs(dx)+abs(dy))
        sy = self._static_map.info.resolution * dy/(abs(dx)+abs(dy))

        while (sx == 0 or xA*self._sign(sx) < xB*self._sign(sx)) and (sy == 0 or yA*self._sign(sy) < yB*self._sign(sy)):
            if self._static_map_cell(xA,yA) > 0.12:
                return True
            xA += sx
            yA += sy
        return False

    def _doughnut(self, p, c, mu, sigma):
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        prob = math.e ** (-((d - mu) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        return prob

    def _uniform_doughnut(self, p, c, mu, sigma):
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        if (d > mu-2*sigma and d < mu+2*sigma) and not self._checkCollision(p,c):
                return 100
        return 0

    def _banana(self, p, c, angle, arclen, mu, sigma):
        gamma = angle - arclen/2.0
        theta = gamma + arclen
        k = math.atan2(c[0]-p[0], c[1]-p[1])  # angle of point p
        #if k >= gamma and k <= theta:
        if k < gamma:
            probangle = math.e ** (-((k - gamma) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        elif k > theta:
            probangle = math.e ** (-((k - theta) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        else:
            probangle = 1 / math.sqrt(2 * math.pi * sigma ** 2)
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        prob = math.e ** (-((d - mu) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        return prob*probangle

    def _first_quadrant(self, a):
        if a < math.pi/2:
            return a
        elif a < math.pi:
            return math.pi - a
        elif a < 3*math.pi/2:
            return math.pi + x
        return 2*math.pi-a

    def _uniform_banana(self, p, c, angle, arclen, mu, sigma):
        gamma = (angle+arclen / 2.0)
        theta = -(gamma)
        k = math.atan2(c[0] - p[0], c[1] - p[1])  # angle of point p

        dist = 2*sigma

        #gp = (c[0] - mu * math.cos(gamma-angle), c[1] - mu * math.sin(gamma-angle))
        #tp = (c[0] + mu * math.cos(theta-angle), c[1] + mu * math.sin(theta-angle))

        #gp = (c[0] + mu * math.sin(gamma-angle), c[1] + mu * math.cos(gamma-angle))
        #tp = (c[0] + mu * math.sin(theta-angle), c[1] + mu * math.cos(theta-angle))

        # gp = (c[0] - abs(mu * math.sin(gamma - angle)), c[1] - abs(mu * math.cos(gamma - angle)))
        # tp = (c[0] - abs(mu * math.sin(theta + angle)), c[1] + abs(mu * math.cos(theta + angle)))
        #
        # gp = (c[0] - (mu * math.sin(gamma)), c[1] - (mu * math.cos(gamma))) # correct 0 angle
        # tp = (c[0] - (mu * math.sin(theta)), c[1] - (mu * math.cos(theta))) # correct 0 angle
        #
        # gp = ((mu * math.sin(gamma)), (mu * math.cos(gamma)))
        # tp = ((mu * math.sin(theta)), -(mu * math.cos(theta)))
        # tp = (c[0] - (tp[0] * math.cos(-angle) + tp[1]*math.sin(-angle)), c[1] - (tp[0]*math.sin(-angle) - tp[1]*math.cos(-angle)))
        # gp = (c[0] - (gp[0] * math.cos(-angle) + gp[1]*math.sin(-angle)), c[1] - (gp[0]*math.sin(-angle) - gp[1]*math.cos(-angle)))

        if k <= gamma and k >= theta:
            return self._uniform_doughnut(p, c, mu, sigma)
        # d = math.sqrt((p[0] - gp[0]) ** 2 + (p[1] - gp[1]) ** 2)
        # r = d <= dist
        # if not r:
        #     d = math.sqrt((p[0] - tp[0]) ** 2 + (p[1] - tp[1]) ** 2)
        #     r = d <= dist
        # return 100*r
        return 0

    def gen_costmap(self):

        self.peaks = []
        self.doughnuts = []
        self.bananas = []

        if rospy.has_param('~peaks'):
            self.peaks = rospy.get_param('~peaks')
        if rospy.has_param('~doughnuts'):
            self.doughnuts = rospy.get_param('~doughnuts')
        if rospy.has_param('~bananas'):
            self.bananas = rospy.get_param('~bananas')

        # Wait for map
        while not rospy.is_shutdown() and not self._static_map:
            rospy.sleep(0.5)

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.info = self._static_map.info

            maxc = 1
            for y in range(grid.info.height):
                for x in range(grid.info.width):
                    cost = 0
                    for elem in self.doughnuts:
                        posx = elem['x']; posy = elem['y']; r = elem['radius']; std_dev = elem['std_dev']
                        cost += self._uniform_doughnut((x*grid.info.resolution, y*grid.info.resolution), (posx, posy), r, std_dev)
                    for elem in self.bananas:
                        posx = elem['x']; posy = elem['y']; r = elem['radius']; a = elem['angle']; al = elem['arclen']; std_dev = elem['std_dev']
                        cost += self._uniform_banana((x*grid.info.resolution, y*grid.info.resolution), (posx, posy), a, al, r, std_dev)
                    if cost > maxc:
                        maxc = cost
                    grid.data.append(cost)

            grid.data = map(lambda c: int(100.0*c/float(maxc)), grid.data)

            self.costmap_pub.publish(grid)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('hidden_costmap')
    try:
        h = HiddenCostMap()
        h.gen_costmap()
    except rospy.ROSInterruptException:
        pass
