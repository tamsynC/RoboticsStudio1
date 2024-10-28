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
    useSimTime = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    
    # filepaths
    robotPackageDir = get_package_share_directory('turtlebot3_gazebo')
    nav2Dir = get_package_share_directory('nav2_bringup')
    gazeboDir = get_package_share_directory('gazebo_ros')
    k10Dir = get_package_share_directory('k10')
    world = os.path.join(
        k10Dir,
        'world',
        'disaster.world'
    )
    rvizConfigFile = os.path.join(
        k10Dir,
        'rviz',
        'config.rviz'
    )
    nav2Params = os.path.join(
        k10Dir,
        'config',
        'nav2_params.yaml'
    )
    slamParams = os.path.join(
        get_package_share_directory('k10'),
        'config',
        'mapper_params_online_async.yaml'
    )
    mapFile = os.path.join(
        k10Dir,
        'map',
        'map.yaml'
    )

    # nodes
    HQNode = Node(
        package="k10",
        executable="k10",
        name="HQ",
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
        arguments=["-d", rvizConfigFile]
    )
    
    # navigation / localisation
    slamNode = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slamParams, {'use_sim_time': useSimTime}]
    )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2Dir, '/launch/bringup_launch.py']),
        launch_arguments={
            'map' : mapFile,
            'use_sim_time': useSimTime,
            'params_file' : nav2Params
        }.items()
    )

    # gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazeboDir,
                         'launch',
                         'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazeboDir,
                         'launch',
                         'gzclient.launch.py'
            )
        )
    )

    # robot
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotPackageDir,
                         'launch',
                         'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': useSimTime
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotPackageDir,
                         'launch',
                         'spawn_turtlebot3.launch.py'
            )
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
    ld.add_action(slamNode)
    #ld.add_action(HQNode)
    #ld.add_action(AudioDetectionV2)
    #ld.add_action(personDetection)
    #ld.add_action(WorkingGas)
    ld.add_action(rvizNode)
    ld.add_action(nav2)
    
    return ld