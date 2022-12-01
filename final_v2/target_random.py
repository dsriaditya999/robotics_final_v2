#! /usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node         import Node
from numpy import random
import numpy as np

from rclpy.qos              import QoSProfile, DurabilityPolicy
from math import (pi, sin, cos, asin, acos, atan2)

from geometry_msgs.msg      import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl

from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from std_msgs.msg       import Bool
from geometry_msgs.msg       import Pose



#
#  Interactive Vector Marker Server
#
#  Create an interactive marker, forming a fixed-length vector and
#  rotating like a pan/tilt gimbal.  Note the vector is the x-axis in
#  the rotated frame.
#
class TargetMarker:
    def __init__(self, node):
        # Store the reference to the node.
        #super().__init__('target_marker')
        self.node  = node

        mark = Marker()
        mark.id = 0
        mark.ns = 'random_marker'
        mark.action = Marker.ADD
        mark.type               = Marker.MESH_RESOURCE
        mark.mesh_resource      = "package://final_v2/meshes/textured.dae"
        mark.mesh_use_embedded_materials = True
        # mark.type = Marker.MESH_RESOURCE
        # mark.mesh_resource = "package://final_v1/meshes/Rock.stl"
        # mark.type = 1
        # mark.color.a = 1.
        # mark.color.r = 0.
        # mark.color.g = 1.
        # mark.color.b = 1.

        mark.header.frame_id = "/world"
        mark.scale.x = 2.0
        mark.scale.y = 2.0
        mark.scale.z = 2.0
        
        mark.pose.position.x = 0.
        mark.pose.position.y = 0.
        mark.pose.position.z = 0.
        mark.pose.orientation.x = 0.
        mark.pose.orientation.y = 0.
        mark.pose.orientation.z = -0.216
        mark.pose.orientation.w = 0.976

        self.marker = mark

    def set_pose(self, pose):
        self.marker.pose = pose



    def process(self, msg):
        # Save the direction contained in the message.  Find the
        # x-axis in the rotated frame (given by the quaternions).
        o = msg.pose.orientation

        # Report (to show working)
        #self.node.get_logger().info("Not Functional")


#
#   Target Node Class
#
class TargetNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Report.
        self.get_logger().info("Starting the interactive marker...")
        

        # Create the interactive vector marker server.
        self.target = TargetMarker(self)

        self.publisher_ = self.create_publisher(Marker, 'random_target', 10)
        self.sub_ = self.create_subscription(Pose, 'set_random_target', self.target.set_pose, 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Reach Target Signal
        # self.subscription_2 = self.create_subscription(
        #     Bool,
        #     'reach',
        #     self.reach_control_rcvd,
        #     10
        # )

    # def reach_control_rcvd(self,msg):

    #     print('Should I reach for the target? "%s"' % msg.data)

    #         # Do something here


    def timer_callback(self):
        # pass
        self.publisher_.publish(self.target.marker)



def main(args=None):
    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress = True, precision = 6)

    # Initialize ROS and the demo node.
    rclpy.init(args=args)
    node = TargetNode('target_node')

    # Run until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

