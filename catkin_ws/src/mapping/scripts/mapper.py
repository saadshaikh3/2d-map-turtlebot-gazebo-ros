#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/scan
/odom

Published topics:
/map 
/map_metadata
/cmd_vel

"""
import math
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np



class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=-5, origin_y=-5, resolution=.1, 
                 width=100, height=100):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        #print('-----------')
        
        #print (flat_grid)
        flat_grid=flat_grid.astype('int8')
      
        ##########


        grid_msg.data = list(np.round(flat_grid))
       

        return grid_msg

    def get_indix(self, x, y):
        x = x - self.origin_x
        y = y - self.origin_y
        i = int(round(x / self.resolution))
        j = int(round(y / self.resolution))
        return i, j


class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    
    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.get_rotation)


        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it. 
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata', 
                                             MapMetaData, latch=True)
        
        # Initialize motion control variables
        self.move = Twist()
        rospy.spin()

    def get_rotation(self, msg):
        global x_r, y_r, yaw_r
        position = msg.pose.pose.position
        x_r, y_r = position.x, position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw_r) = euler_from_quaternion(orientation_list)
        print("x_r: ", x_r, "y_r: ", y_r, "yaw_r: ", yaw_r)
    

    def scan_callback(self, scan):
        """ Update the map on every scan callback. """
        print('--------------------------------')
        print ('the length of the range array is: ')
        print (len(scan.ranges))
        print ('angle_min=', scan.angle_min)
        print('angle_max=', scan.angle_max)
        ###############updat the mape based on scan reading########## 
        #you need to writr your code heer to update your map based on
        #sensor data

        # Obstacle avoidance logic
        if scan.ranges[0] > 0.5:  # No obstacle in front
            self.move.linear.x = 0.5
            self.move.angular.z = 0.0
        else:  # Obstacle detected, turn
            self.move.linear.x = -0.5
            self.move.angular.z = 0.5
        self._cmd_vel_pub.publish(self.move)

        for i, distance in enumerate(scan.ranges):
            if scan.range_min < distance < scan.range_max:
                # Convert polar to Cartesian in local frame
                theta_s = scan.angle_min + i * scan.angle_increment
                x_s = distance * math.cos(theta_s)
                y_s = distance * math.sin(theta_s)

                # Transform to global frame
                x_global = x_r + (x_s * math.cos(yaw_r) - y_s * math.sin(yaw_r))
                y_global = y_r + (x_s * math.sin(yaw_r) + y_s * math.cos(yaw_r))

                # Update the grid cell for this obstacle
                i_cell, j_cell = self._map.get_indix(x_global, y_global)

                # Ensure indices are within bounds
                if 0 <= i_cell < self._map.height and 0 <= j_cell < self._map.width:
                    self._map.grid[i_cell, j_cell] = 1.0


        ############################################
         # Now that the map was updated, so publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()


    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""