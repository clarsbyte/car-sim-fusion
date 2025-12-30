from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import os
from launch.actions import GroupAction, IncludeLaunchDescription

def generate_launch_description():
    car_controller_pkg = get_package_share_directory("car_controller")
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Radius of the wheels'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Distance between the wheels'
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        'use_simple_controller',
        default_value='False',
        description='Whether to use the simple controller or not'
    )

    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')  
    use_simple_controller = LaunchConfiguration('use_simple_controller')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["car_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                           "--controller-manager", 
                           "/controller_manager"
                ]
            ),
            Node(
                package="car_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,}]
            ),
        ]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", "/twist_mux/cmd_vel")],
        parameters=[
            os.path.join(car_controller_pkg, "config", "twist_mux.yaml"),
            os.path.join(car_controller_pkg, "config", "twist_mux_locks.yaml"),
            {"use_sim_time": True},
        ]
    )

    twist_relay_node = Node(
        package="car_controller",
        executable="twist_relay.py",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
        twist_mux_node,
        twist_relay_node
    ])