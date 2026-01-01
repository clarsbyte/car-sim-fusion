# Car Simulator - Gazebo & ROS2 (Vanilla VLA frorm scratch)
Demo Video (click it):
[![Watch the video](https://img.youtube.com/vi/ONvRc97oi6g/maxresdefault.jpg)](https://youtu.be/ONvRc97oi6g)

## Brain Behind VLA
I tried some ollama VLMs on my computer, and the best performing one so far (that my GPU can still handle) is the Llama 3.2 Vision model. While finetuning with a dataset is more effective, my next steps are adding a memory agent and waypoints to this project, since my GPU can only handle 69% of the local mLLM.

Also, one might ask:
Why not use a cloud VLM? 
_Local is free LOL!_

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

To launch safety_stop node:
`ros2 run car_utils safety_stop.py`

### Run the VLA
Ensure you have ollama llama3.2-vision:11b or the model of your liking, just modify mllm.py in the car_mllm package
`ollama run llama3.2-vision:11b`

Run the node:
`ros2 run car_mllm process --ros-args -p target:="i want to go to the weightlifting barbell"`

### Nav 2

`ros2 launch car_localization global_localization.launch.py`

Configure:
`ros2 lifecycle set /map_server 1`

Activate:
`ros2 lifecycle set /map_server 3`


## Learnings
- Ensure the /clock is published on gz_ros2_bridge: `"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",` as it is crucial for twist_mux

## Acknowledgments
Mesh Credit - Antonio Brandi
