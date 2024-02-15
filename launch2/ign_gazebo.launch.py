import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context: LaunchContext):
    print()
    filename = 'robot.urdf.xacro'
    pkg_name = 'osprey_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    classic = eval(context.perform_substitution(LaunchConfiguration('classic')).title())
    which_gazebo = 'gazebo' if classic else 'ign_gazebo'
    world_file = context.perform_substitution(LaunchConfiguration('world')) + '.world'
    world = os.path.join(pkg_path,'worlds', world_file)
    xacro_file = os.path.join(pkg_path,'description',filename)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_hardware:=",
            which_gazebo,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    if which_gazebo == 'gazebo' :
        gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                        launch_arguments={
                            'pause' : 'true',
                            'world' : world,
                        }.items(),
                )

        create_entity = Node(package='gazebo_ros',
                             executable='spawn_entity.py',
                             arguments=['-topic', '/robot_description',
                                        '-entity', 'robot'],
                             output='screen')

    else :
        gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
                        launch_arguments={
                            'pause' : 'true',
                            'ign_args' : world,
                        }.items(),
                    )

        create_entity = Node(package='ros_gz_sim',
                            executable='create',
                            arguments=['-topic', '/robot_description',
                                       '-entity', 'robot'],
                            output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    diff_drive_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='both',
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    position_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state', 'active',
             'position_controllers'],
        output='screen'
    )

    velocity_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state', 'active',
             'velocity_controllers'],
        output='screen'
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_spawner],
        )
    )

    delayed_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[velocity_spawner],
        )
    )

    return [
        node_robot_state_publisher,
        node_joint_state_publisher,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_position_spawner,
        delayed_velocity_spawner,
        gazebo,
        create_entity,
    ]

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "classic",
            default_value="False",
            description="Start classic Gazebo instead of IGN Gazebo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty",
            description="The world the robot will be spawned within.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
