import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import atan2, sqrt
import time

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)

    def move_to_goal(self, current_x, current_y, current_z, target_x, target_y, target_z):
        # 목표 위치와 현재 위치의 차이 계산
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        distance = sqrt(dx**2 + dy**2 + dz**2)  # 3D 거리 계산
        angle = atan2(dy, dx)  # 목표 각도 계산 (x, y 평면에서)

        # 목표 각도에 맞게 회전
        self.get_logger().info(f"목표 각도: {angle:.2f} 라디안으로 회전합니다.")
        self.rotate(angle)

        # 목표 거리만큼 직선 이동
        self.get_logger().info(f"목표 거리: {distance:.2f}m 이동합니다.")
        self.move_straight(distance)

        # 이동 완료 후 정지
        self.stop_robot()
        self.get_logger().info("목표 지점에 도달했습니다.")

    def rotate(self, angle, speed=0.5):
        msg = Twist()
        duration = abs(angle / speed)  # 회전 시간 계산
        msg.angular.z = speed if angle > 0 else -speed  # 양수면 반시계, 음수면 시계 방향 회전
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_robot()

    def move_straight(self, distance, speed=0.2):
        msg = Twist()
        duration = distance / speed  # 직선 이동 시간 계산
        msg.linear.x = speed  # 선속도
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_robot()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    
    # 현재 위치와 목표 위치 설정
    current_x = -2.589092
    current_y = 0.465936
    current_z = 0.008886  # 현재 z 좌표 추가
    target_x = -4.770244
    target_y = 0.404005
    target_z = 0.008886  # 목표 z 좌표 추가
    
    node.move_to_goal(current_x, current_y, current_z, target_x, target_y, target_z)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
