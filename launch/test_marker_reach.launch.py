"""boolean_publisher.launch.py

   Launch the boolean_publisher demo.  This is only intended
   demonstrate the example.  Please edit/update as appropriate.

   This starts:
   1) The boolean_publisher.py
   2) A "ros2 topic echo /boolean" commmand

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import ExecuteProcess
from launch.actions                    import RegisterEventHandler
from launch.actions                    import Shutdown
from launch.event_handlers             import OnProcessExit
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

    # Configure a node for the boolean_publisher.
    node_publisher = Node(
        name       = 'reach',
        package    = 'final_v2',
        executable = 'reach_target_control',
        #output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for the boolean_publisher.
    node_marker = Node(
        name       = 'test_marker_reach',
        package    = 'final_v2',
        executable = 'test_marker',
        #output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
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
    node_joint_gui = Node(
        name       = 'joint_state_publisher_gui', 
        package    = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        output     = 'screen',
        on_exit    = Shutdown())



    # # Run the ros2 topic echo command:
    # sleep_before_echo = ExecuteProcess(
    #     cmd = ['sleep', '2'])

    # cmd_echo = ExecuteProcess(
    #     cmd    = ['ros2', 'topic', 'echo', '/boolean'],
    #     output = 'screen')

    # delay_echo = RegisterEventHandler(
    #     event_handler = OnProcessExit(
    #         target_action = sleep_before_echo,
    #         on_exit       = [cmd_echo]))



    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Register the delayed events first.
        #delay_echo,

        # Start the boolean pulisher and the topic echo.
        node_publisher,
        #sleep_before_echo,
        node_marker,
        node_rviz,
        node_urdf,
        node_joint_gui,

    ])
