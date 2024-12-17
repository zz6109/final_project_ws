import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String  # String 메시지 타입 추가
from geometry_msgs.msg import TransformStamped

class PositionPublisherNode(Node):
    def __init__(self):
        super().__init__('position_publisher_node')
        
        # TF2 버퍼와 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 위치를 발행할 퍼블리셔 생성 (String 타입)
        self.position_publisher = self.create_publisher(String, '/position', 10)
        
        # 타이머 설정: 3초마다 위치 업데이트
        self.timer = self.create_timer(3.0, self.publish_position)  # 주기를 3.0초로 설정
        
        # 기준 프레임 설정
        self.map_frame = 'map'        # Cartographer의 맵 프레임
        self.robot_frame = 'base_link'  # 로봇의 프레임
        
    def publish_position(self):
        try:
            # map -> base_link 변환 가져오기
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())

            # 위치 정보 가져오기 (z 제외)
            position = transform.transform.translation
            x, y = position.x, position.y

            # String 메시지로 변환
            position_str = f"x={x:.2f}, y={y:.2f}"
            msg = String()
            msg.data = position_str

            # 메시지 발행
            self.position_publisher.publish(msg)
            self.get_logger().info(f'Published Position: {position_str}')

        except Exception as e:
            self.get_logger().warning(f'Failed to get transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
