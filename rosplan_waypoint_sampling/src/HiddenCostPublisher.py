#!/usr/bin/env python
import rospy
import math
from math import floor
from itertools import product
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

class HiddenCostMap:
    def __init__(self):
        self._static_map = None
        self._hidden_costmap = None
        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10, latch=True)
        self.hppits_pub = rospy.Publisher('hppits_map', OccupancyGrid, queue_size=10, latch=True)
        self.mergemap_pub = rospy.Publisher('merged_map', OccupancyGrid, queue_size=10, latch=True)

        self.peaks = self.doughnuts = self.bananas = self.hppits = []
        if rospy.has_param('~peaks'):
            self.peaks = rospy.get_param('~peaks')
        if rospy.has_param('~doughnuts'):
            self.doughnuts = rospy.get_param('~doughnuts')
        if rospy.has_param('~bananas'):
            self.bananas = rospy.get_param('~bananas')
        self.hppits = []
        if rospy.has_param('~hppits'):
            self.hppits = rospy.get_param('~hppits')

        rospy.Subscriber("map", OccupancyGrid, self.set_map)
        self.obj_pub = rospy.Publisher('objects', MarkerArray, queue_size=10, latch=True)
        self.publish_objects()

    def set_map(self, msg):
        self._static_map = msg

    def _static_map_cell(self, x, y):
        gx = int((x - self._static_map.info.origin.position.x) / self._static_map.info.resolution)
        gy = int((y - self._static_map.info.origin.position.y) / self._static_map.info.resolution)
        return self._static_map.data[gx + gy * self._static_map.info.width]

    def _sign(self, n):
        return (n > 0) - (n < 0)

    # return if there is a collision between points p and c
    def _checkCollision(self, p, c):

        (xA, yA) = (p[0], p[1])
        (xB, yB) = (c[0], c[1])
        (dx, dy) = (xB - xA, yB - yA)
        sx = self._static_map.info.resolution * dx / (abs(dx) + abs(dy))
        sy = self._static_map.info.resolution * dy / (abs(dx) + abs(dy))

        while (sx == 0 or xA * self._sign(sx) < xB * self._sign(sx)) and (
                sy == 0 or yA * self._sign(sy) < yB * self._sign(sy)):
            if self._static_map_cell(xA, yA) > 0.12:
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
        if (d > mu - 2 * sigma and d < mu + 2 * sigma) and not self._checkCollision(p, c):
            return 100
        return 0

    def _uniform_hppit(self, p, c, mu, sigma):
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        if (d > mu - 2 * sigma and d < mu + 2 * sigma):
            return 0
        return 1

    def _banana(self, p, c, angle, arclen, mu, sigma):
        gamma = angle - arclen / 2.0
        theta = gamma + arclen
        k = math.atan2(c[0] - p[0], c[1] - p[1])  # angle of point p
        # if k >= gamma and k <= theta:
        if k < gamma:
            probangle = math.e ** (-((k - gamma) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        elif k > theta:
            probangle = math.e ** (-((k - theta) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        else:
            probangle = 1 / math.sqrt(2 * math.pi * sigma ** 2)
        d = math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2)
        prob = math.e ** (-((d - mu) ** 2) / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)
        return prob * probangle

    def _first_quadrant(self, a):
        if a < math.pi / 2:
            return a
        elif a < math.pi:
            return math.pi - a
        elif a < 3 * math.pi / 2:
            return math.pi + x
        return 2 * math.pi - a

    def _uniform_banana(self, p, c, angle, arclen, mu, sigma):
        gamma = (angle + arclen / 2.0)
        theta = -(gamma)
        k = math.atan2(c[0] - p[0], c[1] - p[1])  # angle of point p

        dist = 2 * sigma

        if k <= gamma and k >= theta:
            return self._uniform_doughnut(p, c, mu, sigma)
        return 0

    def gen_costmap(self, preferences=False):

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
            object_grid = OccupancyGrid()
            object_grid.header.frame_id = "map"
            object_grid.info = self._static_map.info
            hppit_grid = OccupancyGrid()
            hppit_grid.header.frame_id = "map"
            hppit_grid.info = self._static_map.info

            omaxc = 1
            hmaxc = 1
            for y in range(object_grid.info.height):
                for x in range(object_grid.info.width):
                    obj_cost = 0
                    hppit_cost = 1
                    for elem in self.doughnuts:
                        posx = elem['x'];
                        posy = elem['y'];
                        r = elem['radius'];
                        std_dev = elem['std_dev']
                        obj_cost += self._uniform_doughnut((x * object_grid.info.resolution, y * object_grid.info.resolution), (posx, posy), r, std_dev)
                    for elem in self.bananas:
                        posx = elem['x'];
                        posy = elem['y'];
                        r = elem['radius'];
                        a = elem['angle'];
                        al = elem['arclen'];
                        std_dev = elem['std_dev']
                        obj_cost += self._uniform_banana((x * object_grid.info.resolution, y * object_grid.info.resolution), (posx, posy), a, al, r, std_dev)
                    for elem in self.hppits:
                        posx = elem['x'];
                        posy = elem['y'];
                        r = elem['radius'];
                        std_dev = elem['std_dev']
                        hppit_cost *= self._uniform_hppit((x * object_grid.info.resolution, y * object_grid.info.resolution), (posx, posy), r, std_dev)

                    if obj_cost > omaxc:
                        omaxc = obj_cost

                    if hppit_cost > hmaxc:
                        hmaxc = hppit_cost
                    object_grid.data.append(obj_cost)
                    hppit_grid.data.append(hppit_cost)

            #object_grid.data = map(lambda c: int(100.0 * c / float(omaxc)), object_grid.data)
            object_grid.data = map(lambda c: int(100.0 * ((omaxc-c) if c > 0 else 0) / float(omaxc)), object_grid.data)
            #hppit_grid.data = map(lambda c: int(100.0 * (c == hmaxc)), hppit_grid.data)

            self.costmap_pub.publish(object_grid)
            self.hppits_pub.publish(hppit_grid)
            self.mergemaps(object_grid, hppit_grid)
            rate.sleep()


    def mergemaps(self, object_map, hiddenpref_map):
        merged = OccupancyGrid()
        merged.header.frame_id = "map"
        merged.info = self._static_map.info
        for y in range(object_map.info.height):
            for x in range(object_map.info.width):
                v = (int(object_map.data[y*object_map.info.width + x]) * int(hiddenpref_map.data[y*hiddenpref_map.info.width + x]))
                #v = v/100
                merged.data.append(v)
        self.mergemap_pub.publish(merged)


    def publish_objects(self):
        ma = MarkerArray()
        i = 0
        for o in self.peaks:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "taws_objects"
            marker.id = i
            i+= 1
            marker.action = Marker.ADD
            marker.pose.position.x = o['x']
            marker.pose.position.y = o['y']
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.type = Marker.SPHERE;
            ma.markers.append(marker)

        for o in self.doughnuts:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "taws_objects"
            marker.id = i
            i+= 1
            marker.action = Marker.ADD
            marker.pose.position.x = o['x']
            marker.pose.position.y = o['y']
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.type = Marker.SPHERE;
            ma.markers.append(marker)
   
        for o in self.bananas:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "taws_objects"
            marker.id = i
            i+= 1
            marker.action = Marker.ADD
            marker.pose.position.x = o['x']
            marker.pose.position.y = o['y']
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.type = Marker.SPHERE;
            ma.markers.append(marker)

        for o in self.hppits:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "taws_objects"
            marker.id = i
            i+= 1
            marker.action = Marker.ADD
            marker.pose.position.x = o['x']
            marker.pose.position.y = o['y']
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.type = Marker.SPHERE
            ma.markers.append(marker)
        self.obj_pub.publish(ma)

if __name__ == '__main__':
    rospy.init_node('hidden_costmap')
    try:
        h = HiddenCostMap()
        h.gen_costmap(preferences=False)
    except rospy.ROSInterruptException:
        pass
