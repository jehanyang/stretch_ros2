"""
Simulation Room Navigation Launch File

This launch file starts all components needed for room navigation in MuJoCo simulation:
1. MuJoCo simulation (stretch_mujoco_driver)
2. Multi-goal control (da_sim)
3. Driver assistance with shared control (da_core)
4. Room navigator with navigation and cmd_vel mux (stretch_nav2)

Usage:
    ros2 launch stretch_nav2 sim_room_navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml

    # With custom robocasa layout/style:
    ros2 launch stretch_nav2 sim_room_navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml robocasa_layout:=<layout> robocasa_style:=<style>
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_nav2_path = get_package_share_directory('stretch_nav2')

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stretch_nav2_path, 'map', 'home2.yaml'),
        description='Full path to the map.yaml file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to launch RViz'
    )

    use_mujoco_viewer_arg = DeclareLaunchArgument(
        'use_mujoco_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to show MuJoCo viewer'
    )

    shared_arg = DeclareLaunchArgument(
        'shared',
        default_value='true',
        description='Use shared controller (true) or teleop only (false)'
    )

    robocasa_layout_arg = DeclareLaunchArgument(
        'robocasa_layout',
        default_value='Random',
        description='RoboCasa kitchen layout'
    )

    robocasa_style_arg = DeclareLaunchArgument(
        'robocasa_style',
        default_value='Random',
        description='RoboCasa kitchen style'
    )

    # 1. MuJoCo simulation
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stretch_simulation'), '/launch/stretch_mujoco_driver.launch.py'
        ]),
        launch_arguments={
            'use_mujoco_viewer': LaunchConfiguration('use_mujoco_viewer'),
            'use_rviz': 'false',  # We'll use nav2's rviz
            'mode': 'navigation',
            'robocasa_layout': LaunchConfiguration('robocasa_layout'),
            'robocasa_style': LaunchConfiguration('robocasa_style'),
        }.items()
    )

    # 2. Multi-goal control (da_sim)
    multi_goal_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('da_sim'), '/launch/multi_goal_ctrl.launch.py'
        ])
    )

    # 3. Driver assistance (da_core)
    driver_assistance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('da_core'), '/launch/driver_assistance.launch.py'
        ]),
        launch_arguments={
            'shared': LaunchConfiguration('shared'),
        }.items()
    )

    # 4. Room navigator (includes navigation_with_mux) - delayed to allow simulation to start
    room_navigator_launch = TimerAction(
        period=3.0,  # Wait 3 seconds for simulation to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    stretch_nav2_path, '/launch/room_navigator.launch.py'
                ]),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'true',
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'teleop_type': 'none',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        use_rviz_arg,
        use_mujoco_viewer_arg,
        shared_arg,
        robocasa_layout_arg,
        robocasa_style_arg,
        mujoco_launch,
        multi_goal_ctrl_launch,
        driver_assistance_launch,
        room_navigator_launch,
    ])
