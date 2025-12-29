# Car Simulator - Gazebo & ROS2

## Things to run

Simulation:
`ros2 launch car_description gazebo.launch.py`

Controller:
`ros2 launch car_controller controller.launch.py`

Keyboard Teleop:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/car_controller/cmd_vel -p stamped:=true`