import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from custom_interface.srv import ArmService
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
            self.get_logger().info('Waiting for \"arm_control_service\" to become available...')

        # 로봇 상태
        self.current_angle = math.radians(180)  
        self.max_angular_velocity = 0.1  

    def angle_callback(self, msg):
        target_angle = math.radians(msg.data) 
        self.get_logger().info(f'Received target angle: {msg.data} degrees')

        angle_diff = (target_angle - self.current_angle) % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi

        # 방향 설정 및 회전
        if abs(angle_diff) > 1e-2:
            direction = 1 if angle_diff > 0 else -1
            angular_velocity = direction * self.max_angular_velocity

            # 회전 시간 계산
            rotation_time = abs(angle_diff) / self.max_angular_velocity

            self.get_logger().info(f'Starting rotation for {rotation_time:.2f} seconds at {angular_velocity:.3f} rad/s')

            start_time = time.time()
            while time.time() - start_time < rotation_time:
                self.publish_velocity(angular_velocity)
                time.sleep(0.1)

            self.publish_velocity(0.0)  # 회전 멈춤

            self.call_service('mode2')
            self.get_logger().info('Rotation complete.')

            self.current_angle = target_angle
        else:
            self.publish_velocity(0.0)
            self.get_logger().info('Target angle matches current orientation. Staying still.')

    def publish_velocity(self, angular_velocity):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self.publisher_.publish(twist)

    def call_service(self, mode):
        request = ArmService.Request()
        request.mode = mode

        self.get_logger().info(f'Calling service with mode: {mode}')
        future = self.cli.call_async(request) # 비 동기
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """서비스 호출 후 콜백 함수."""
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    angle_to_cmd_vel = AngleToCmdVel()
    try:
        rclpy.spin(angle_to_cmd_vel)
    except KeyboardInterrupt:
        pass
    finally:
        angle_to_cmd_vel.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
