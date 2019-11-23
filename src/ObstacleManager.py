#!/usr/bin/env python
# import cv2
import math
import numpy
import Utils
import rospy
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt

MAP_TOPIC = 'static_map'


class ObstacleManager(object):
    def __init__(self, mapMsg, car_width, car_length, collision_delta):
        self.map_info = mapMsg.info
        self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
            (mapMsg.info.height, mapMsg.info.width, 1))

        # Retrieve the map dimensions
        height, width, channels = self.mapImageGS.shape
        self.mapHeight = height
        self.mapWidth = width
        self.mapChannels = channels

        # Binarize the Image
        self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
        self.mapImageBW[self.mapImageGS == 0] = 0
        self.mapImageBW[400:, 400:] = 1  # set an area of collision
        print '  init: mapImageBW:', self.mapImageBW.shape

        # Obtain the car length and width in pixels
        self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
        self.robotLength = int(car_length / self.map_info.resolution + 0.5)
        self.collision_delta = collision_delta

    # Check if the passed config is in collision
    # config: The configuration to check (in meters and radians)
    # Returns False if in collision, True if not in collision
    def get_state_validity(self, config):

        # Convert the configuration to map-coordinates -> mapConfig is in pixel-space
        mapConfig = Utils.world_to_map(config, self.map_info)
        print '  >> car_position_px: ', mapConfig

        # ---------------------------------------------------------
        # YOUR CODE HERE
        #
        # Return true or false based on whether the robot's configuration is in collision
        # Use a square to represent the robot, return true only when all points within
        # the square are collision free
        #
        # Also return false if the robot is out of bounds of the map
        #
        # Although our configuration includes rotation, assume that the
        # square representing the robot is always aligned with the coordinate axes of the
        # map for simplicity
        # ----------------------------------------------------------

        # test for boundary
        # I don't know why here, if the index is out of the boundary of mapImageBW, no error will occur
        min_x = mapConfig[0] - self.robotWidth / 2
        max_x = mapConfig[0] + self.robotWidth / 2 + 1
        min_y = mapConfig[1] - self.robotWidth / 2
        max_y = mapConfig[1] + self.robotWidth / 2 + 1

        if min_x < 0 or min_y < 0 or max_x < 0 or max_y < 0:
            print '  >> State_Validity: Min Out for Map!'
            return False

        map_square = self.mapImageBW[min_x:max_x, min_y:max_y, 0]
        if map_square.shape != (self.robotWidth, self.robotLength):
            print '  >> State_Validity: Max Out for Map!'
            return False

        # test for collision
        if numpy.sum(map_square):
            print '  >> State_Validity: Collision!', map_square
            return False

        return True

    # Discretize the path into N configurations, where N = path_length / self.collision_delta
    # input: an edge represented by the start and end configurations
    # return three variables:
    # list_x - a list of x values of all intermediate points in the path
    # list_y - a list of y values of all intermediate points in the path
    # edgeLength - The euclidean distance between curr and config2
    def discretize_edge(self, curr, target):
        list_x, list_y = [], []
        edgeLength = 0

        # -----------------------------------------------------------
        # YOUR CODE HERE
        # -----------------------------------------------------------
        theta = numpy.arctan2(target[1] - curr[1], target[0] - curr[0])
        x = curr[0]
        y = curr[1]
        while edgeLength <= numpy.sqrt(numpy.square(curr[0] - target[0]) + numpy.square(curr[1] - target[1])):
            edgeLength += self.collision_delta
            x += self.collision_delta * numpy.cos(theta)
            y += self.collision_delta * numpy.sin(theta)
            list_x.append(x)
            list_y.append(y)

        return list_x, list_y, edgeLength

    # Check if there is an unobstructed edge between the passed configs
    # curr, target: The configurations to check (in meters and radians)
    # Returns false if obstructed edge, True otherwise
    def get_edge_validity(self, curr, target):
        # -----------------------------------------------------------
        # YOUR CODE HERE
        #
        # Check if endpoints are obstructed, if either is, return false
        # Find path between two configs by connecting them with a straight line
        # Discretize the path with the discretized_edge function above
        # Check if all configurations along path are obstructed
        # -----------------------------------------------------------
        list_x, list_y, _ = self.discretize_edge(curr, target)
        for i in range(len(list_x)):
            print self.get_state_validity((list_x[i], list_y[i], 0))
            if not self.get_state_validity((list_x[i], list_y[i], 0)):
                return False
        return True


# Write Your Test Code For Debugging
if __name__ == '__main__':
    mapMsg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map
    car_width, car_length, collision_delta = 0.33, 0.33, 0.05
    om = ObstacleManager(mapMsg, car_width, car_length, collision_delta)

    # small_basement :: 2792x1236, resolution = 0.02, so the test position should be [0,0] to [55.84, 24,72]
    # gates :: 3168x1984
    # simples :: 800x800, offset = 8 meters, so [-8,-8] to [8,8], 255 from [] to []
    # print om.get_state_validity([0, 0, 0])
    # print om.get_state_validity([-1, -1, 0])
    # print om.get_state_validity([-8, -8, 0])
    print om.get_edge_validity([-1, -1, 0], [-4.1, -4.1, 0])

    pass
