#!/usr/bin/env python
import rospy
import math  
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

    def _doughnut(self, p, c, mu, sigma):
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        prob = math.e ** (-((d - mu) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        return prob

    def _banana(self, p, c, angle, arclen, mu, sigma):
        gamma = angle - arclen/2.0
        theta = gamma + arclen
        k = math.atan2(c[0]-p[0], c[1]-p[1])
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

        rate = rospy.Rate(0.05)
        while not rospy.is_shutdown():
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.info = self._static_map.info

            maxc = 0
            for y in range(grid.info.height):
                for x in range(grid.info.width):
                    cost = 0
                    for (posx, posy, r, std_dev) in self.doughnuts:
                        cost += self._doughnut((x*grid.info.resolution, y*grid.info.resolution), (posx, posy), r, std_dev) / len(self.doughnuts)
                    for (posx, posy, r, a, al, std_dev) in self.bananas:
                        cost += self._banana((x*grid.info.resolution, y*grid.info.resolution), (posx, posy), a, al, r, std_dev) / (len(self.bananas) + len(self.doughnuts))
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
