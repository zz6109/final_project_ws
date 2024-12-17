import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from custom_interface.srv import ArmService
from playsound import playsound


class CustomControlNode(Node):
    def __init__(self):
        super().__init__('custom_control')

        self.mission_publisher = self.create_publisher(String, 'mission', 10)

        self.detection_subscription = self.create_subscription(
            String,
            '/detection_result',
            self.detection_callback,
            10
        )

        self.position_subscription = self.create_subscription(
            String,
            '/position',
            self.position_callback,
            10
        )

        self.cli = self.create_client(ArmService, 'arm_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "arm_control_service" to become available...')
        
        # self.value = None  

    def detection_callback(self, msg):
        """Callback function for /detection_result topic."""
        message = msg.data
        self.get_logger().info(f'Message received: {message}')

        if message == 'person':
            self.true_condition()
        elif message == 'object':
            self.false_condition()

    def position_callback(self, msg):
        """Callback function for /position topic to update current position."""
        self.value = msg.data
        self.get_logger().info(f'Current position updated: {self.value}')

    def true_condition(self):
        """Actions to perform if the message is 'person'."""
        self.get_logger().info('Executing True condition...')
        self.play_mp3("/home/autocar/warning_sound.mp3")  # 경고음 재생

        # 미션 완료 메시지 발행
        if self.value is not None:
            msg = String()
            msg.data = f"complete {self.value}"
            self.mission_publisher.publish(msg)
            self.get_logger().info(f"Mission complete message published with position: {self.value}")
        else:
            self.get_logger().warn("Position not yet received. Unable to publish complete message.")

        # mode3 서비스 호출
        self.call_service("mode3")

    def false_condition(self):
        """Actions to perform if the message is 'object'."""
        self.get_logger().info('Executing False condition...')
        self.call_service("mode1")  # mode1 서비스 호출

    def play_mp3(self, file_path):
        """Play an MP3 file."""
        try:
            self.get_logger().info(f"MP3 파일 재생: {file_path}")
            playsound(file_path)
        except Exception as e:
            self.get_logger().error(f"MP3 재생 오류: {e}")

    def call_service(self, mode):
        """Call arm_control_service with a specific mode."""
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available!')
            return
        
        request = ArmService.Request()
        request.mode = mode
        
        self.get_logger().info(f'Calling service with mode: {mode}')
        future = self.cli.call_async(request) # 비동기 처리
        
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """Callback function after the service response."""
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CustomControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
