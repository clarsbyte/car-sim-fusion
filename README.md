# Car Simulator - Gazebo & ROS2

## Things to run

Simulation:
`ros2 launch car_description gazebo.launch.py`
For other worlds, example in a small house:
`ros2 launch car_description gazebo.launch.py world_name:=small_house`

Controller:
`ros2 launch car_controller controller.launch.py`

Keyboard Teleop:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/car_controller/cmd_vel -p stamped:=true`
Twist Mux Running:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/teleop/cmd_vel`

If want to see odom in Rviz2: 
`ros2 launch car_description display.launch.py`

To see IMU output (/imu/out) as a plot, you can use Foxglove: 
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

Sensor fusion for odom and IMU (Comprehension purposes)
`ros2 run car_localization kalman_filter.py`

## Major Learnings
- Ensure the /clock is published on gz_ros2_bridge: `"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",` as it is crucial for twist_mux

## Acknowledgments
Mesh Credit - Antonio Brandi