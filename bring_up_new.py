# micro_ros_agent_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rplidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros'), 'launch'),
         '/rplidar_a1_launch.py'])
      )
   
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent1',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB1']
        ),       

        Node(
            package='motor_control',            
            executable='motor_control_node',
            name='motor_control'
        ),
        Node(
            package='robot_odom',            
            executable='odom',
            name='odometry'
        ),        
        rplidar
    ])
