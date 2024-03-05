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
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    # Set default arguments
    current_pkg = FindPackageShare('dlio')
    map_handler_pkg = get_package_share_directory("map_handler")
    pointcloud_topic = LaunchConfiguration('pointcloud_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    dlio_yaml_name = LaunchConfiguration('dlio_yaml_name').perform(context)
    dlio_params_name = LaunchConfiguration('dlio_params_name').perform(context)
    dlio_yaml_name +=".yaml"
    dlio_params_name +=".yaml"
    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', dlio_yaml_name])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', dlio_params_name])



    map_handler_pkg = os.path.join(map_handler_pkg, 'pcd')

    param_substitutions = {
        'save_map_path': map_handler_pkg}

    configured_dlio_yaml_path = RewrittenYaml(
            source_file=dlio_yaml_path,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    
    dlio_nodes = GroupAction(
        actions=[   
                # DLIO Odometry Node
                Node(
                    package='dlio',
                    executable='dlio_odom_node',
                    output='screen',
                    parameters=[configured_dlio_yaml_path, dlio_params_yaml_path],
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
                    parameters=[configured_dlio_yaml_path, dlio_params_yaml_path],
                    remappings=[
                        ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),

                    ],
                )
                ]
    )


    static_world_to_map_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
    )
    
    # static_base_link_to_fake_laser =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_link_to_fake_laser',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'fake_laser']
    # )
    # static_base_footprint_to_lidar  =  Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name="base_footprint_imu_link",
    #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link lidar_link'.split(' '),
    #     output='screen',
    #     )

    delayed_dlio_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[dlio_nodes],
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

    return [
        static_world_to_map_node,
        delayed_dlio_server,
        # static_base_footprint_to_lidar,
        # static_base_link_to_fake_laser,
        rviz_node
    ]

def generate_launch_description():



    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='points_raw',
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu/data',
        description='IMU topic name'
    )
    declare_dlio_yaml_name_arg = DeclareLaunchArgument(
        'dlio_yaml_name',
        default_value='dlio',
        description='dlio_yaml_name'
    )
    declare_dlio_params_name_arg = DeclareLaunchArgument(
        'dlio_params_name',
        default_value='params',
        description='dlio_params_name'
    )



    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_dlio_yaml_name_arg,
        declare_dlio_params_name_arg,
        OpaqueFunction(function=launch_setup)
    ])
