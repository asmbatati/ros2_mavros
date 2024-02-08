from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard_control',
            executable='gt_x500_tf',
            name='gt_x500_tf_node',
            output='screen',
            remappings=[
                ('/pose_array', '/yolo_detections_poses')
            ],
            parameters=[
                {'parent_frame': 'interceptor/odom'},
                {'child_frames': ['x500/base_link']}
            ]
        )
    ])
