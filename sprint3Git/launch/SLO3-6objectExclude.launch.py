from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        #start the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('smart_factory'),'launch','factory_world.launch.py'))
        ),

        #start the custom node
        Node(
            package='sprint3',
            executable='objectExcludeExe',
            name='objectExcludeNode',
            output='screen',
        ),

        #uncomment if needed
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
        #     output='screen'
        # ),
    ])























