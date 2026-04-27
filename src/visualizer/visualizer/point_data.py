import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2

import serial
import struct
import time

import random
import numpy as np

## COORDINATE FRAMES
''' 
* Base frame is base of rotating apparatus. Dimensions unknown im gonna kms
* Lidar frame will follow standard for cameras w/ z axis pointing out, y down, 
and x to the left

* for now im going to assume lidar camera is 0.1 m up the base frame z axis
'''

## TRANSFORMS
'''
* 0T1 = [0 0 1 0; -1 0 0 0; 0 -1 0 h; 0 0 0 1]
where 1 is the lidar camera frame and 0 is the base frame

* 1T2 (transformation from static end camera joint to camera frame) =
[c(t), s(t)s(p), s(t)c(p), 0; 0, c(p), -s(p), 0; -s(t), -s(t)s(p), c(t)c(p), 0; 0, 0, 0, 1]

* To express a depth d at point P in the world frame, 0P=0T1*1T2*2P where 2P = [0;0;0;d;1]
'''

# global variables
final_encoder = 3400 # total encoder count
h = 0.1 # distance frame base to camera in meters
base_trans = np.array([[0,0,1,0],
                       [-1,0,0,0],
                       [0,-1,0,h],
                       [0,0,0,1]])

''' ok so plan is to use two functions to define position, theta for the angle
around the y axis, and phi for the angle around the x axis'''

'''simple model where theta goes from 0 to 2pi and phi is +/- pi/4 as a wave'''

# open rvix with command rviz2 and then select /point_cloud topic to view point cloud

class PointData(Node):
    def __init__(self):
        super().__init__('point_data')

        # connect to arduino
        # try:
        #     self.arduino = serial.Serial("dev/ttyACMO", timeout=1, baudrate= 115200) # try high baud rate, will need short cable
        # except:
        #     print("cannot connect to arduino")

        # create publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.timer = self.create_timer(0.1, self.publish_points)

        # initialize list of points
        self.data = []
        self.points = []
        # state variables
        self.publish = False

        ## CREATE FAKE POINTS
        self.depth = [random.uniform(0.2, 5.0) for d in range(final_encoder)]
        self.encoder_counts = [i for i in range(final_encoder)]
        self.theta = np.linspace(0, 2*np.pi, final_encoder)
        self.phi = np.linspace(-np.pi/4, np.pi/4, final_encoder)
        self.data = self.encoder_counts


        # self.read_values()
        self.encoder_conversion()

    def read_values(self):

        encoder_count = 0

        while encoder_count < (0.90 * self.final_encoder):
            # 4 bytes float, 4 bytes long: float,long
            data = self.arduino.read(8)

            # if the length isnt long enough, skip
            if len(data) != 8:
                continue

            # unpack data
            depth, encoder_count = struct.unpack('<fl')
            self.data.append([depth, encoder_count])
            print("depth:", depth, "encoder count:", encoder_count)

        self.encoder_conversion()


    def encoder_conversion(self):
        # self.data = [depth, encoder_count]

        # conversion here
        for i in self.data:
            # encoder to theta, phi conversion here
            # for now ->
            theta = self.theta[i]
            phi = self.phi[i]
            depth = self.depth[i]

            # call depth_to_camera conversion
            self.depth_to_camera(theta, phi, depth)

        self.publish = True
        

    def depth_to_camera(self, theta, phi, depth):
        # coordinate transdform from static frame 1 to moving camera frame 2
        ct = np.cos(theta)
        cp = np.cos(phi)
        st = np.sin(theta)
        sp = np.sin(phi)

        dynamic_camera_trans = np.array([[ct, st*sp, st*cp, 0],
                                        [0, cp, -sp, 0],
                                        [-st, -st*sp, ct*cp, 0],
                                        [0, 0, 0, 1]])
        
        # transform from camera camera frame to world frame, 0T1*1T2
        world_trans = base_trans @ dynamic_camera_trans


        camera_depth = np.array([[0],[0],[depth],[1]])

        # conversion to world cartesian point
        world_depth = world_trans @ camera_depth

        # make a list to append to points list
        x = world_depth[0]
        y = world_depth[1]
        z = world_depth[2]
        self.points.append([x, y, z])

    def publish_points(self):
        if self.publish:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'

            msg = point_cloud2.create_cloud_xyz32(header, self.points)

            self.cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = PointData()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

