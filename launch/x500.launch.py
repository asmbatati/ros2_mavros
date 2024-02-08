#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 2
    model_name = {'gz_model_name': 'x500'}
    autostart_id = {'px4_autostart_id': '4021'}
    instance_id = {'instance_id': '2'}
    xpos = {'xpos': '4.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.1'}
    headless= {'headless' : '0'}

    # Namespace
    ns='x500'

    # PX4 SITL + Spawn x3
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_offboard_control'),
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': '0.0'
        }.items()
    )

    # MAVROS
    file_name = 'x500_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('px4_offboard_control')
    plugins_file_path = os.path.join(package_share_directory, file_name)
    file_name = 'x500_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory, file_name)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_offboard_control'),
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace' :ns+'/mavros',
            'tgt_system': '3',
            'fcu_url': 'udp://:14542@127.0.0.1:14559',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'x500/base_link',
            'odom_frame': 'x500/odom',
            'map_frame': 'map'
        }.items()
    )    

    odom_frame = 'odom'
    base_link_frame=  'base_link'

    # Static TF map(or world) -> local_pose_ENU
    map_frame = 'map'
    odom_frame= 'odom'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0', '0', '0', map_frame, ns+'/'+odom_frame],
    )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.15
    cam_y = 0.0
    cam_z = 0.25
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns+'_base2depth_tf_node',
        executable='static_transform_publisher',
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link_frame, 'x500_d435_1/link/realsense_d435'],
        
    )

    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_depthcam',
        executable='parameter_bridge',
        arguments=['/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '--ros-args', '-r', '/d435/depth_image:='+ns+'/depth_image',
                   '-r', '/d435/image:='+ns+'/image',
                   '-r', '/d435/points:='+ns+'/points',
                   '-r', '/d435/camera_info:='+ns+'/camera_info'
                   ],
    )

    
    offboard_control_node = Node(
        package='px4_offboard_control',
        executable='offboard_control',
        output='screen',
        name='offboard_node',
        namespace=ns,
        parameters=[ {'trajectory_type': 'infty'},
                    {'system_id': 3},
                    {'radius': 3.0},
                    {'omega': 0.5},
                    {'normal_vector': [1.0, 1.0, 1.0]},
                    {'center': [10.0, 0.0, 10.0]},
        ],
        remappings=[
            ('mavros/state', 'mavros/state'),
            ('mavros/local_position/odom', 'mavros/local_position/odom'),
            ('mavros/setpoint_raw/local', 'mavros/setpoint_raw/local')
        ]
    )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('px4_offboard_control'), 'sim.rviz')]
    )

    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(rviz_node)
    ld.add_action(offboard_control_node)
    ld.add_action(mavros_launch)

    return ld