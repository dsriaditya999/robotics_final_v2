# 133a Robotics Final Project

For this problem, we are considering a simulated environment where there are obstacles (e.g. rocks) falling vertically. The robot being used is a redundant snake arm. The robotic snake arm has to perform the following tasks :

- Avoid the obstacles hitting its body.
- Touch a random target (e.g. a colored cube) in the workspace, with the correct gripper orientation.
- If initially entangled in a loop, it has to untangle itself and then repeat, the above mentioned tasks.

This project use ```ROS2``` to simulate the robot movement, and the demo link is [here](https://youtu.be/g8eaxnVIguQ).

To run the entire simulation, run the script
```
ros2 launch final_v2 task4.launch.py
```
The script should 
- Open RVIZ and show the XY robot with 41 DOFs
- Repeat the movement for 4 periods, each period contains tasks as follows
    - Stay in a knot for 1 second
    - Untie itself while avoiding falling obstacles for 4 seconds
    - Touch targets while avoiding falling obstacles for 20 seconds
- Plot the cumulative number of touched target, the cumulative number of obstacle collision and the noncumulative number of self-collision

If you only want to see the performance of a single task, run the following script(s)
To see the simulation of touching target while avoiding falling rocks, run
```
ros2 launch final_v2 task2.launch.py
```
To see the simulation of knot untying, run
```
ros2 launch final_v2 task3.launch.py
```
