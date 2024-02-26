import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context: LaunchContext):

    filename = 'robot.urdf.xacro'
    pkg_name = 'osprey_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
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
                        get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                        launch_arguments={
                            'pause' : 'true',
                            'gz_args' : world,
                        }.items(),
                    )

        create_entity = Node(package='ros_gz_sim',
                            executable='create',
                            arguments=['-topic', '/robot_description',
                                       '-entity', 'robot'],
                            output='screen')
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
                ("/diff_drive_controller/odom", "/odom"),
            ],
        )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "-c", "/controller_manager"],
    )

    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "-c", "/controller_manager"],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[ slam_params_file, {'use_sim_time': True} ],
    )

    static_transform_publisher_spawner = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "opsrey_ros/guide_frame/lidar_sensor"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    delayed_slam_toolbox_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[slam_toolbox_node],
        )
    )

    delayed_static_transform_publisher_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_spawner],
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

    nodes = [
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
        delayed_slam_toolbox_node_spawner,
        delayed_static_transform_publisher_spawner,
        delayed_diff_drive_spawner,
        delayed_position_spawner,
        delayed_velocity_spawner,
        gazebo,
        create_entity,
    ]

    if which_gazebo != 'gazebo' :
        nodes = nodes + [bridge]

    return nodes

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
