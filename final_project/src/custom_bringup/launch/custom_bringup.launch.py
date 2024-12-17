from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # arm_service_server 노드 실행
        Node(
            package='custom_bringup',
            executable='arm_service_server',
            name='arm_service_server',
            output='screen'
        ),

        # sound_angle_pub 노드 실행
        Node(
            package='custom_bringup',
            executable='sound_angle_pub',
            name='sound_angle_pub',
            output='screen'
        ),

        # sound_angle_sub 노드 실행
        Node(
            package='custom_bringup',
            executable='sound_angle_sub',
            name='sound_angle_sub',
            output='screen'
        ),

        # realsense2_camera 노드 실행
        Node(
            package="realsense2_camera", executable="realsense2_camera_node",
        parameters=[
            {"enable_color": True},
            {"enable_depth": False},
            {"enable_infra1": False},
            {"enable_infra2": False},
            {"rgb_camera.profile": "640x480x30"},
            {"rgb_camera.power_line_frequency": 2}
        ], 
        ),
    ])
