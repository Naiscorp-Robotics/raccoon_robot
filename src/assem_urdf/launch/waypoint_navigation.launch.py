#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Khai báo arguments
    script_type_arg = DeclareLaunchArgument(
        'script_type',
        default_value='simple',
        description='Loại script: simple hoặc advanced'
    )
    
    # Đường dẫn đến package
    pkg_share = FindPackageShare('assem_urdf')
    
    # Chọn script dựa trên argument
    script_type = LaunchConfiguration('script_type')
    
    # Node chạy script waypoint navigation
    waypoint_node = Node(
        package='assem_urdf',
        executable='waypoint_navigation.py',
        name='waypoint_navigator',
        output='screen',
        condition=lambda context: context.launch_configurations['script_type'] == 'advanced'
    )
    
    # Node chạy script test đơn giản
    simple_waypoint_node = Node(
        package='assem_urdf',
        executable='simple_waypoint_test.py',
        name='simple_waypoint_test',
        output='screen',
        condition=lambda context: context.launch_configurations['script_type'] == 'simple'
    )
    
    return LaunchDescription([
        script_type_arg,
        waypoint_node,
        simple_waypoint_node,
    ]) 