# probabilistic grid map
Creating probabilistic grid map in ROS2 with Gazebo and Rviz2.

Probabilistic Grid Map built based on sensor data from turtlebot3 burger model. Map is being published to /map topic and visualised in Rviz2 with pose of the robot.

to run:

<code>ros2 launch mapping mapping.launch.py</code>

in Rviz2:

Fixed frame: <code>map</code> 

Map topic: <code>/map</code>

Marker topic: <code>/robot_marker</code>
