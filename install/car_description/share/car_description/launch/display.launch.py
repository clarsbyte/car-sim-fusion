from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("car_description")
            , "urdf", "robot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str) # read model argument, if none fallback goes to model_arg

    # clarissa@clar-ubuntu:~/Documents/pers-robotics/car-sim-fusion$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro //home/clarissa/Documents/pers-robotics/car-sim-fusion/src/car_description/urdf/robot.urdf.xacro)"
    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description
                    }
                ]
            ) 
    
    joint_state_publisher_gui = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(
                    get_package_share_directory("car_description"),
                    "rviz",
                    "display.rviz",
                )],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz_node
    ])