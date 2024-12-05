import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleBotVoiceControl(Node):
    def __init__(self):
        super().__init__('turtlebot_voice_control')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("TurtleBot Voice Control Node Initialized!")

    def send_rotation_command(self, doa):
        twist = Twist()

        if doa < 180:
            # 시계 방향으로 회전
            twist.angular.z = -0.5 
        else:
            # 반시계 방향으로 회전
            twist.angular.z = 0.5 

        duration = abs(doa - 180) / 90.0 
        self.cmd_publisher.publish(twist)
        self.get_logger().info(f"TurtleBot rotating to DOA: {doa}° for {duration:.2f} seconds")
        time.sleep(duration)

        # 정지 
        twist.angular.z = 0.0 
        self.cmd_publisher.publish(twist)
        self.get_logger().info("TurtleBot stopped")

def simulate_doa():
    return random.randint(0, 359)


def simulate_audio_recognition():
    phrases = ["안녕하세요", "왼쪽입니다", "오른쪽입니다"]
    return random.choice(phrases)


def main():
    rclpy.init()
    turtlebot_control = TurtleBotVoiceControl()

    try:
        while rclpy.ok():
            # 가짜 DOA 값 생성
            doa = simulate_doa()
            print(f"Simulated DOA: {doa}°")

            turtlebot_control.send_rotation_command(doa)

            # 가짜 음성 데이터 처리
            text = simulate_audio_recognition()
            print(f"Simulated Recognized Text: {text}")

            time.sleep(1)  
    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
