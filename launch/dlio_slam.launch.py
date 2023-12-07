#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('dlio')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='points_raw')
    imu_topic = LaunchConfiguration('imu_topic', default='imu/data')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    dlio_nodes = GroupAction(
        actions=[   
                # DLIO Odometry Node
                Node(
                    package='dlio',
                    executable='dlio_odom_node',
                    output='screen',
                    parameters=[dlio_yaml_path, dlio_params_yaml_path],
                    remappings=[
                        ('pointcloud', pointcloud_topic),
                        ('imu', imu_topic),
                        ('odom', 'dlio/odom_node/odom'),
                        ('pose', 'dlio/odom_node/pose'),
                        ('path', 'dlio/odom_node/path'),
                        ('kf_pose', 'dlio/odom_node/keyframes'),
                        ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                        ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
                    ],
                ),
                # DLIO Mapping Node
                Node(
                    package='dlio',
                    executable='dlio_map_node',
                    output='screen',
                    parameters=[dlio_yaml_path, dlio_params_yaml_path],
                    remappings=[
                        ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
                    ],
                )
                ]
    )

    occupied_cell_node = Node(
            package='occupied_grid_publisher',
            executable='occupied_grid_publisher',
            name='occupied_grid_publisher',
            output="screen"
    )


    fake_scan_move = Node(
        package='fake_frame',
        executable='fake_scan',
        name='fake_base_link',
            parameters=[{'target_topic': "scan_for_move"}]    
    )

    fake_scan = Node(
        package='fake_frame',
        executable='fake_scan',
        name='fake_base_link',
            parameters=[{'target_topic': "scan"}]    
    )


    static_world_to_map_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
    )
    
    static_base_link_to_fake_laser =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_fake_laser',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'fake_laser']
    )
    static_base_footprint_to_lidar  =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="base_footprint_imu_link",
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_footprint lidar_link'.split(' '),
        output='screen',
        )

    static_map_to_odom_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    delayed_dlio_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[dlio_nodes],
        )
    )


    delayed_fake_map_to_odom =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[static_map_to_odom_node],
        )
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        static_world_to_map_node,
        delayed_dlio_server,
        static_base_footprint_to_lidar,
        fake_scan_move,
        fake_scan,
        occupied_cell_node,
        static_base_link_to_fake_laser,
        delayed_fake_map_to_odom,
        rviz_node
    ])
