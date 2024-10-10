from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    nav2_bringup_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # Create the launch description to include navigation_launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items()  # Pass the argument 'use_sim_time:=True'
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        #launch slam toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Set to True if using simulation (Gazebo)
            }],
        ),

        #launch nav2
        #ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=True'],
            output='screen'
        ),

        #launch world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('smart_factory'),'launch','factory_world.launch.py'))
        ),

        #launch teleop keybard
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen'
        ),

        # rvis launch,
        #ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'rviz2', 'rviz2', '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
            output='screen'
        ),

        Node(
            package='sprint3',
            executable='mapOverlayExe',
            name='mapOverlayExeNode',
            output='screen',
        ),
    ])