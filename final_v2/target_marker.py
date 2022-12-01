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



        # Create a marker of type cube for the vector, unrotated.
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.header.stamp       = node.get_clock().now().to_msg()
        self.marker.ns                 = "target"
        self.marker.id                 = 0
        #self.marker.type               = Marker.CUBE
        self.marker.type               = Marker.MESH_RESOURCE
        self.marker.mesh_resource      = "package://final_v2/meshes/textured.dae"
        self.marker.mesh_use_embedded_materials = True
        self.marker.action             = Marker.ADD
        self.marker.pose.orientation.w =      0.976
        self.marker.pose.orientation.x =   0.0
        self.marker.pose.orientation.y =   0.
        self.marker.pose.orientation.z =   -0.216
        self.marker.pose.position.x    =   0.
        self.marker.pose.position.y    =   0.
        self.marker.pose.position.z    =   0.
        self.marker.scale.x            =   2.0
        self.marker.scale.y            =   2.0
        self.marker.scale.z            =   2.0
        # self.marker.color.r            =   0.             
        # self.marker.color.g            =   0.
        # self.marker.color.b            =   1.0
        # self.marker.color.a            =   1.0             # Make solid
            

        # Create an interactive marker for our server at the position
        # and with the initial orientation.
        self.imarker = InteractiveMarker()
        self.imarker.header.frame_id = "world"
        self.imarker.name = "target"
        self.imarker.pose.orientation.w =     0.976
        self.imarker.pose.orientation.x =   0.
        self.imarker.pose.orientation.y =   0.
        self.imarker.pose.orientation.z =   -0.216
        self.imarker.pose.position.x    =   0.
        self.imarker.pose.position.y    =   0.
        self.imarker.pose.position.z    =   0.
        self.imarker.scale = 0.2

        # Append a non-interactive control which contains the cube
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.marker)
        self.imarker.controls.append(control)

        # Append an interactive control for Z rotation

        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = sin(0.25*pi)
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)

        # Append an interactive control for Y rotation.

        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = sin(0.25*pi)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)

        # Append an interactive control for X rotation.

        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = sin(0.25*pi)
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)

        # Append an interactive control for X movement

        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = sin(0.25*pi)
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)

        # Append an interactive control for Y movement.

        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = sin(0.25*pi)
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)
        
        # Append an interactive control for Z movement.

        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = sin(0.25*pi)
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.imarker.controls.append(control)

        # Create an interactive marker server on the topic namespace
        # ring, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'target')
        server.insert(self.imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Cube Interactive Marker and Subscriber set up...")




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


        self.publisher_ = self.create_publisher(Pose, 'target_marker_pub', 10)

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
        self.publisher_.publish(self.target.imarker.pose)



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

