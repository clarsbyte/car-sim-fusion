from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    car_description_dir = get_package_share_directory("car_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        car_description_dir, "urdf", "robot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_env = [
        SetEnvironmentVariable("IGN_PARTITION", "local"),
        SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", 
                              "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
    ]

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="SDF world file name"
    )

    world_path = PathJoinSubstitution([
        car_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'" ])
    ])

    model_path = str(Path(car_description_dir).parent.resolve())
    model_path += os.pathsep + os.path.join(car_description_dir, "models")

    # clarissa@clar-ubuntu:~/Documents/pers-robotics/car-sim-fusion$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro //home/clarissa/Documents/pers-robotics/car-sim-fusion/src/car_description/urdf/robot.urdf.xacro)"
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path
    )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )    


    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", PythonExpression(expression=(["'", world_path, " -v 4 -r'"])))
                ]
             )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "car"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/imu", "/imu/out"),
        ],
    )

    return LaunchDescription([
        *gazebo_env,
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])