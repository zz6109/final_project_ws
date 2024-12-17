import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from custom_interface.srv import ArmService
from playsound import playsound


class CustomControlNode(Node):
    def __init__(self):
        super().__init__('custom_control')

        # Publisher for /mission topic
        self.mission_publisher = self.create_publisher(String, 'mission', 10)

        # Subscriber for /detection_result
        self.detection_subscription = self.create_subscription(
            String,
            '/detection_result',
            self.detection_callback,
            10
        )

        # Subscriber for /position
        self.position_subscription = self.create_subscription(
            String,
            '/position',
            self.position_callback,
            10
        )

        self.cli = self.create_client(ArmService, 'arm_control_service')
        self.value = None  # 변수에 로봇의 위치를 저장하기 위한 초기화

    def detection_callback(self, msg):
        """Callback function for /detection_result topic."""
        message = msg.data
        self.get_logger().info(f'Message received: {message}')

        if message == 'person':
            self.true_condition()
        elif message == 'object':
            self.false_condition()

    # 다른 노드에서 로봇의 위치를 /position이라는 토픽으로 지속발행중-이때의 토픽을 구독해서 변수 value에 저장
    def position_callback(self, msg):
        """Callback function for /position topic to update current position."""
        self.value = msg.data
        self.get_logger().info(f'Current position updated: {self.value}')

    def true_condition(self):
        """Actions to perform if the message is 'person'."""
        self.get_logger().info('Executing True condition...')
        # 경고음 출력: 요구조자 발견X3
        self.play_mp3("/home/autocar/warning_sound.mp3")
        # 미션 완료 및 현재 위치좌표(value) 라는 토픽발행
        if self.value is not None:
            msg = String()
            msg.data = f"complete {self.value}"
            self.mission_publisher.publish(msg)
            self.get_logger().info(f"Mission complete message published with position: {self.value}")
        else:
            self.get_logger().warn("Position not yet received. Unable to publish complete message.")

        # mode3 서비스콜 및 커스텀 브링업으로 생성된 모든 노드 종료
        self.call_mode_service("mode3")

    def false_condition(self):
        """Actions to perform if the message is 'object'."""
        self.get_logger().info('Executing False condition...')
        # mode1 서비스콜
        self.call_mode_service("mode1")
        # /cmd_vel, twist.linear.x 0.5로 전진 벽나올때 까지(send_goal)

    def play_mp3(self, file_path):
        try:
            from playsound import playsound
            self.get_logger().info(f"MP3 파일 재생: {file_path}")
            playsound(file_path)
        except ImportError:
            self.get_logger().error("playsound 라이브러리가 설치되어 있지 않습니다.")
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
          
    def call_mode_service(self, mode):
        request = ArmService.Request()
        request.data = mode
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.mode_service_callback)

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
