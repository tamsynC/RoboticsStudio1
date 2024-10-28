#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # settings
    # starting pose
    x_pose = LaunchConfiguration('x_pose', default='9')
    y_pose = LaunchConfiguration('y_pose', default='-9')
    
    # launch parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_lifestyle_autostart = LaunchConfiguration('autostart', default='true')
    use_map_subscribe = LaunchConfiguration('map_subscribe_transient_local', default='false')
    lifecycle_nodes = ['map_server', 'amcl']
    
    # filepaths
    robotLaunch = get_package_share_directory('turtlebot3_gazebo')
    nav2Launch = get_package_share_directory('nav2_bringup')
    world = os.path.join(
        get_package_share_directory('k10'),
        'world',
        'disaster.world'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('k10'),
        'rviz',
        'test_config.rviz'
    )
    slam_params_file = os.path.join(
        get_package_share_directory('k10'),
        'config',
        'my_ma_mapper_params.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('k10'),
        'map',
        'my_ma.yaml'
    )

    # nodes
    HQNode = Node(
        package="k10",
        executable="k10",
        name="HQ",
        arguments=["exec", "HQ"]
    )
    
    AudioDetectionV2 = Node(
        package="k10",
        executable="k10",
        name="AudioDetectionV2",
        output="screen"
    )
    
    personDetection = Node(
        package="k10",
        executable="k10",
        name="personDetection",
        output="screen"
    )
    
    WorkingGas = Node(
        package="k10",
        executable="k10",
        name="WorkingGas",
        output="screen"
    )

    rvizNode = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )
    
    # navigation / localisation
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'use_sim_time': use_sim_time}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'map': map_file},
                    {'use_sim_time': use_sim_time}]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2Launch, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'map_subscribe_transient_local': use_map_subscribe,
                          'use_sim_time': use_sim_time
        }.items()
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # fixes terminal output https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': use_lifestyle_autostart},
                    {'node_names': lifecycle_nodes}])

    # gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # robot
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotLaunch, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotLaunch, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # add actions to launch
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(HQNode)
    ld.add_action(AudioDetectionV2)
    ld.add_action(personDetection)
    ld.add_action(WorkingGas)
    ld.add_action(rvizNode)
    ld.add_action(slam_node)
    ld.add_action(nav2)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_manager_cmd)

    return ld