import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from custom_service.srv import ArmService
import math
import time

class AngleToCmdVel(Node):
    def __init__(self):
        super().__init__('sound_angle_sub')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Int32,
            '/detect_angle',
            self.angle_callback,
            10
        )
        self.cli = self.create_client(ArmService, 'arm_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for \'arm_control_service\' to become available...')

        # 로봇 상태
        self.current_angle = math.radians(180)  # 현재 각도 (180도 고정, 라디안)
        self.max_angular_velocity = 0.1  # 최대 각속도 (rad/s)

    def angle_callback(self, msg):
        target_angle = math.radians(msg.data)  # 입력 받은 각도를 라디안으로 변환
        self.get_logger().info(f'Received target angle: {msg.data} degrees')

        angle_diff = (target_angle - self.current_angle) % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi

        # 방향 설정 및 회전
        if abs(angle_diff) > 1e-2:  # 목표 각도와의 차이가 충분히 크면 회전
            direction = 1 if angle_diff > 0 else -1
            angular_velocity = direction * self.max_angular_velocity

            # 회전 시간 계산
            rotation_time = abs(angle_diff) / self.max_angular_velocity

            self.get_logger().info(f'Starting rotation for {rotation_time:.2f} seconds at {angular_velocity:.3f} rad/s')

            start_time = time.time()
            while time.time() - start_time < rotation_time:
                self.publish_velocity(angular_velocity)
                time.sleep(0.1)  # 0.1초 간격으로 업데이트

            self.publish_velocity(0.0)  # 회전 멈춤
            # arm 동작 서비스 콜
            self.call_mode_service()
            self.get_logger().info('Rotation complete.')

            # 현재 각도를 목표 각도로 업데이트
            self.current_angle = target_angle
        else:
            self.publish_velocity(0.0)
            self.get_logger().info('Target angle matches current orientation. Staying still.')

    def publish_velocity(self, angular_velocity):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self.publisher_.publish(twist)

    # 서비스 콜 함수
    def call_mode_service(self):
        request = ArmService.Request()
        request.data = 'mode2'
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.mode_service_callback)

    def mode_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Mode 2 activated successfully.')
            else:
                self.get_logger().info('Failed to activate Mode 2.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    angle_to_cmd_vel = AngleToCmdVel()
    rclpy.spin(angle_to_cmd_vel)
    angle_to_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
