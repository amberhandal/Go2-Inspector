import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('go2_navigation')
    
    # Arguments
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='true')
    
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'go2_navigation.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'slam_view.rviz'])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'publish_frequency': 100.0,
        }]
    )
    
    # Odom TF Bridge
    odom_tf_bridge = Node(
        package='go2_navigation',
        executable='odom_tf_bridge',
        name='odom_tf_bridge',
        output='screen',
        parameters=[{
            'odom_topic': '/utlidar/robot_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    joint_state_bridge = Node(
        package='go2_navigation',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen',
    )
    
    # Static transform: base_link -> utlidar_lidar
    # IMPORTANT: Use command-line arguments, not parameters!
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        output='screen',
        arguments=[
            '--x', '0.28945',
            '--y', '0.0',
            '--z', '-0.046825',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'utlidar_lidar'
        ]
    )
    
    # PointCloud to LaserScan
    pointcloud_to_scan = Node(
        package='go2_navigation',
        executable='pointcloud_to_scan',
        name='pointcloud_to_scan',
        output='screen',
        parameters=[{
            'cloud_topic': '/utlidar/cloud',
            'scan_topic': '/scan',
            'scan_frame': 'utlidar_lidar',
            'min_height': -0.1,
            'max_height': 0.5,
            'range_min': 0.15,
            'range_max': 25.0,
        }]
    )
    
    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')),
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 25.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.3,
            'minimum_travel_heading': 0.3,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 10.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,
            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',
        }]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_slam_arg,
        robot_state_publisher,
        odom_tf_bridge,
        joint_state_bridge,
        base_to_lidar_tf,
        pointcloud_to_scan,
        slam_toolbox,
        rviz,
    ])