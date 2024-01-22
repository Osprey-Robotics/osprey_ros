import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'osprey_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={
                    'pause' : 'true',
                    'world' : os.path.join(pkg_path,'worlds','empty.world'),
                    }.items(),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
