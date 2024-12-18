from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # World 파일 경로
    custom_world_path = '/home/test/Desktop/my_gazebo_H/test.world'

    # Gazebo 서버 실행
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', custom_world_path],
        output='screen'
    )

    # Gazebo 클라이언트 실행
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient
    ])
