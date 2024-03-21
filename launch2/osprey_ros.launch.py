import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_context import LaunchContext
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context: LaunchContext):
    pkg_name = 'osprey_ros'
    filename = 'robot.urdf.xacro'

    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    sim = eval(context.perform_substitution(LaunchConfiguration('sim')).title())
    year = context.perform_substitution(LaunchConfiguration('year')).title()
    controller_params = os.path.join(pkg_path, 'config', 'robot' + year + '_controllers.yaml')
    print("controller_params ", controller_params)
    xacro_file = os.path.join(pkg_path,'description',filename)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_hardware:=",
            "mock" if sim else "robot",
            " ",
            "year:=",
            year,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output='both',
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

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
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

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[position_spawner],
        )
    )

    delayed_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[velocity_spawner],
        )
    )

    nodes = [
        controller_manager,
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_position_spawner,
    ]

    if year == "":
        nodes += [gpio_controller_spawner, delayed_velocity_spawner,]

    return nodes

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="False",
            description="Start with simulated mock hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "year",
            default_value="",
            description="Year of robot to start",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
