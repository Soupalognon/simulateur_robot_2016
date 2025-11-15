#!/usr/bin/env python

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    ROBOT_COLOR = "YELLOW"      #"YELLOW"   "BLUE"

    package_dir = get_package_share_directory('simulateur_robot_2016')
    resource_dir = os.path.join(package_dir, 'resource')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([resource_dir, 'World_CDR_2026', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_spawner = Node(
        package='simulateur_robot_2016',
        executable='robot_spawner',
        output='screen',
        arguments=[
            'Assemblage_Carcasse',
            ROBOT_COLOR,
        ],
    )

    if(ROBOT_COLOR == "YELLOW"):
        mapArguments = ['0.3', '1.775', '0', '-1.57', '0', '0', 'map', 'odom']
    elif(ROBOT_COLOR == "BLUE"):
        mapArguments = ['2.7', '1.775', '0', '-1.57', '0', '0', 'map', 'odom']
    map_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=mapArguments,
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['position_controller'] + controller_manager_timeout,
    )

    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner, position_controller_spawner]

    robot_description_path = os.path.join(resource_dir, 'Robot', 'urdf', 'Assemblage_Carcasse_webots.urdf')
    ros2_control_params = os.path.join(resource_dir, 'Robot', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    robot_driver = WebotsController(
        robot_name='Assemblage_Carcasse',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Wait for the simulation to be ready
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start= ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='World_CDR_2026.wbt',
            description='Choose one of the world files from `/simulateur_robot_2016/resource/worlds` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot_spawner,

        footprint_publisher,
        map_publisher,

        robot_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])
