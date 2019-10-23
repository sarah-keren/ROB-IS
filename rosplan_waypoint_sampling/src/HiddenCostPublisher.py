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
        rospy.Subscriber("map", OccupancyGrid, self.set_map)

    def set_map(self, msg):
        self._static_map = msg

    def _doughnut(self, p, c, mu, sigma):
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        prob    = math.e ** (-((d - mu) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        return prob

    def gen_costmap(self):
        if rospy.has_param('~peaks'):  # In case peaks changed
            self.peaks = rospy.get_param('~peaks')
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
                    for (posx, posy, r) in self.peaks:
                        cost += self._doughnut((x, y), (posx, posy), r, r/2) / len(self.peaks)
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
