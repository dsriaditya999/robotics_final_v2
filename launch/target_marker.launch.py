"""intereactivedemo.py

   Launch the interactive marker demo.  This is only intended
   demonstrate the example.  Please edit/update as appropriate.

   This starts:
   1) The interactivedemo.py
   2) RVIZ to see the visualization markers

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Define the package.
    package = 'final_v2'

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir(package), 'rviz/viewcomp.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir(package), 'urdf/xy_41dof.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()



    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the point_publisher.
    node_target = Node(
        name       = 'target_marker',
        package    = 'final_v2',
        executable = 'target_marker',
        # output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the boolean_publisher.
    node_publisher = Node(
        name       = 'reach',
        package    = 'final_v2',
        executable = 'reach_target_control',
        #output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for Robot URDF
    node_urdf = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        arguments  = [urdf],
        on_exit    = Shutdown())

    # Configure a node for Robot Joint GUI
    # node_joint_gui = Node(
    #     name       = 'joint_state_publisher_gui', 
    #     package    = 'joint_state_publisher_gui',
    #     executable = 'joint_state_publisher_gui',
    #     output     = 'screen',
    #     on_exit    = Shutdown())

    node_joint_test6 = Node(
        name       = 'test6', 
        package    = 'final_v2',
        executable = 'test6',
        output     = 'screen',
        on_exit    = Shutdown())

    node_joint_marker = Node(
        name       = 'marker_orig', 
        package    = 'final_v2',
        executable = 'marker_orig',
        # output     = 'screen',
        on_exit    = Shutdown())

    node_rqt = Node(
        name='rqt_plots',
        package = 'rqt_plot',
        executable = 'rqt_plot',
        output     = 'screen',
        arguments =['/collision/data[0]','/touch_target/data[0]','/touch_target/data[1]'],
        on_exit    = Shutdown())



    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the demo and RVIZ
        node_target,
        node_rviz,
        node_urdf,
        node_publisher,
        node_joint_test6,
        node_joint_marker,
        node_rqt
    ])
