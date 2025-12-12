"""
Navigation launch file with cmd_vel mux.

This is a modified version of navigation.launch.py that includes a cmd_vel_mux node
to pause/resume Nav2 velocity commands via a service.

Usage:
    # Launch navigation with mux
    ros2 launch stretch_nav2 navigation_with_mux.launch.py map:=/path/to/map.yaml

    # Enable Nav2 commands (default)
    ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: true}"

    # Disable Nav2 commands (pause navigation)
    ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: false}"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy


def generate_launch_description():
    logger = rclpy.logging.get_logger('navigation_with_mux_launch')

    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')
    navigation_bringup_path = get_package_share_directory('nav2_bringup')

    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Whether to autostart lifecycle nodes on launch')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stretch_navigation_path, 'map', 'home2.yaml'),
        description='Full path to the map.yaml file to use for navigation')

    use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        choices=['True', 'False'],
        description='Whether run a SLAM')

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    nav2_enabled_param = DeclareLaunchArgument(
        'nav2_enabled',
        default_value='true',
        choices=['true', 'false'],
        description='Whether Nav2 cmd_vel forwarding is initially enabled')

    # Error out if the map file does not exist
    def map_file_check(context: LaunchContext):
        if LaunchConfiguration('use_slam').perform(context):
            return
        map_path = LaunchConfiguration('map').perform(context)
        if not os.path.exists(map_path):
            msg = 'Map file not found in given path: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)
        if not map_path.endswith('.yaml'):
            msg = 'Map file is not a yaml file: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)

    map_path_check_action = OpaqueFunction(function=map_file_check)

    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    # Use the modified bringup that includes the mux
    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/bringup_with_mux.launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'map': LaunchConfiguration('map'),
            'slam': LaunchConfiguration('use_slam'),
            'params_file': LaunchConfiguration('params_file'),
            'nav2_enabled': LaunchConfiguration('nav2_enabled'),
        }.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_bringup_path, '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        teleop_type_param,
        use_sim_time_param,
        autostart_param,
        map_path_param,
        use_slam,
        params_file_param,
        rviz_param,
        nav2_enabled_param,
        stretch_driver_launch,
        rplidar_launch,
        base_teleop_launch,
        navigation_bringup_launch,
        rviz_launch,
        map_path_check_action,
    ])
