import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interface.action import StopOnObstacle
from rclpy.executors import MultiThreadedExecutor
import numpy as np


class StopOnObstacleServer(Node):
    def __init__(self):
        super().__init__('stop_on_obstacle_server')

        # 액션 서버 생성
        self._action_server = ActionServer(
            self,
            StopOnObstacle,
            'stop_on_obstacle',
            self.execute_callback
        )

        # LiDAR 데이터 수신 및 제어 발행
        self.laser_subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 기본 변수 초기화
        self.closest_distance = float('inf')
        self.laser_ranges = []
        self.safe_margin = 0.1  # 안전 거리 설정

    def laser_callback(self, msg):
        """
        LiDAR 데이터 콜백 함수: 가장 가까운 거리를 업데이트합니다.
        """
        # inf 값을 필터링하여 업데이트 + 중간값 필터 적용
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if valid_ranges:
            self.closest_distance = np.median(valid_ranges)
        else:
            self.closest_distance = float('inf')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('StopOnObstacle 액션 실행 중...')
        safe_distance = goal_handle.request.safe_distance
        front_angle_range = goal_handle.request.front_angle_range

        feedback_msg = StopOnObstacle.Feedback()

        # 조건 만족 전까지 전진
        twist_msg = Twist()
        twist_msg.linear.x = 0.05  # 0.05의 속도로 전진
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

        while rclpy.ok():
            # 정면 120도 범위 내 거리 확인
            if not self.laser_ranges:
                continue

            mid_index = len(self.laser_ranges) // 2
            range_indices = int((front_angle_range / 360) * len(self.laser_ranges))
            front_ranges = self.laser_ranges[mid_index - range_indices // 2 : mid_index + range_indices // 2]

            # 유효 거리 확인 및 최소값 계산
            filtered_ranges = [r for r in front_ranges if r > 0.0 and r < float('inf')]
            if filtered_ranges:
                self.closest_distance = np.median(filtered_ranges)
            else:
                self.closest_distance = float('inf')

            feedback_msg.current_distance = self.closest_distance

            # 피드백 전송
            goal_handle.publish_feedback(feedback_msg)

            # 조건 확인: 안전 거리 이내에 장애물 검지 시 정지
            if self.closest_distance <= safe_distance + self.safe_margin:
                self.get_logger().info(f"장애물 검지! 거리: {self.closest_distance:.2f}m")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist_msg)

                goal_handle.succeed()
                result = StopOnObstacle.Result()
                result.obstacle_detected = True
                return result

            # 조건 만족전까지 속도 유지
            self.cmd_vel_publisher.publish(twist_msg)

        # 결과 반환
        result = StopOnObstacle.Result()
        result.obstacle_detected = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StopOnObstacleServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
