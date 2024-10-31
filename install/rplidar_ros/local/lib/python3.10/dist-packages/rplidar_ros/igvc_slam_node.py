#!/usr/bin/env python3

import copy
import numpy as np
import rclpy
import math
import itertools
import time
import sys
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from rclpy.node import Node
from builtin_interfaces.msg import Time 

class Igvc_Slam_Node(Node):
    def __init__(self):
        super().__init__('igvc_slam_node')
        
        # Configuration
        self.wait_for_vision = True

        # Camera Vertical vision (m)
        self.camera_vertical_distance = 2.75

        # Camera Horizontal vision (m)
        self.camera_horizontal_distance = 3

        # Publishers
        self.config_pub = self.create_publisher(OccupancyGrid,"/igvc_slam", 1)

        #Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, "/lidar_map", self.lidar_callback, 10)
        self.vision_sub = self.create_subscription(OccupancyGrid, "/lane_map", self.lanes_camera_callback, 10)

        #Timer
        timer_period=0.1
        self.timer = self.create_timer(timer_period, self.config_space_callback)

        # Configuration space map
        self.origin = Pose()
        self.origin.position.x = -10.0
        self.origin.position.y = -10.0
        self.metadata = MapMetaData(
        map_load_time=Time(sec=int(time.time()), nanosec=int((time.time() % 1) * 1e9)),  # Correctly create the Time message
        resolution=0.1,
        width=100,
        height=100,
        origin=self.origin
        )
        #self.metadata = MapMetaData(map_load_time = time.time(), resolution=0.1,width = 100, height = 100, origin = self.origin)

        self.header = Header()
        self.header.frame_id = "map"

        self.max_range = 0.55 # meters
        self.no_go_percent = 0.75
        self.no_go_range = self.max_range * self.no_go_percent # meters

        self.max_range = int(self.max_range / (self.camera_horizontal_distance / 80))
        self.no_go_range = int(self.no_go_range / (self.camera_horizontal_distance / 80))

        self.xxxs = list(range(-self.max_range, self.max_range + 1))
        self.circle_around_indicies = [(0,0,0)]
        for x in self.xxxs:
            for y in self.xxxs:
                if self.max_range * self.no_go_percent <= math.sqrt(x**2 + y**2) < self.max_range and (x+y)%3==0:
                    self.circle_around_indicies.append((x, y, math.sqrt(x**2 + y**2)))
                # if x == 0 or y == 0:
                #     circle_around_indicies.append((x, y, math.sqrt(x**2 + y**2)))

        # Initializiation
        self.last_lidar = None
        self.last_vision = None

    def lidar_callback(self,data):
        # use the global vars
        self.last_lidar
        self.last_lidar = data


    def lanes_camera_callback(self,data):
        self.last_vision
        self.last_vision = data.data

    # TODO: currently this whole thing is set to use the most recent map frames from each perception unit (which is fine for now). In the future the sizing will change as the map grows.
    def config_space_callback(self):

        if self.last_vision is None and self.last_lidar is None:
            return

        # start = time.time()

        # Reset the hidden layer
        self.config_space = [0] * (80 * 80)

        self.combined_maps = None
        if self.last_vision is not None and self.last_lidar is not None:
            self.combined_maps = [x+y for x,y in zip(self.last_lidar.data, self.last_vision)]
        elif self.last_vision is not None:
            self.combined_maps = self.last_vision
        else:
            self.combined_maps = self.last_lidar.data

        # Update the hidden layer before applying the new map to the current configuration space
        for x in range(80):
            for y in range(1,80):
                if self.combined_maps[x + y * 80] > 0:
                    for x_i, y_i, dist in self.circle_around_indicies:
                            index = (x + x_i) + 80 * (y + y_i)

                            if 0 <= (x + x_i) < 80 and 0 <= (y + y_i) < 80:
                                val_at_index = self.config_space[index]
                                linear_t = 100 - ((dist - self.no_go_range) / (self.max_range - self.no_go_range) * 100)

                                if dist <= self.no_go_range:
                                    # obstacle expansion
                                    self.config_space[index] = 100
                                elif dist <= 100 and val_at_index <= linear_t:
                                    # linearly decay
                                    self.config_space[index] = int(linear_t)

        # Publish the configuration space
        self.header.stamp = self.get_clock().now().to_msg()
        config_msg = OccupancyGrid(header=self.header, info=self.metadata, data=self.config_space)
        self.config_pub.publish(config_msg)

        # rospy.loginfo(f"dilate time: {(time.time() - start)*1000:0.01f}ms")
        sys.stdout.flush()

def main():
    # Set up node
    rclpy.init()

    #Igvc_slam_node object created
    igvc_slam_node=Igvc_Slam_Node()

    # Wait for topic updates
    rclpy.spin(igvc_slam_node)
    
    igvc_slam_node.destroy_node()
    rclpy.shutdown()

# Main setup
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
 



    