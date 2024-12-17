import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import usb.core
import usb.util
import time
from custom_bringup.tuning import Tuning
import speech_recognition as sr
import pyttsx3

class ReSpeakerNode(Node):
    def __init__(self):
        super().__init__('sound_angle_pub')
        
        # ReSpeaker 장치 설정
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

        if not self.dev:
            self.get_logger().error("ReSpeaker 장치를 찾을 수 없습니다.")
            raise RuntimeError("ReSpeaker 장치가 연결되지 않았습니다.")

        # DOA 초기화
        self.mic_tuning = Tuning(self.dev)

        # 음성 인식 객체 생성
        self.recognizer = sr.Recognizer()

        # 텍스트를 음성으로 변환하는 엔진 초기화
        self.engine = pyttsx3.init()

        self.angle_publisher = self.create_publisher(Int32, '/detect_angle', 10)

        # 타이머로 listen_and_respond를 주기적으로 호출
        self.timer = self.create_timer(3.0, self.listen_and_respond)

    def listen_and_respond(self):
        with sr.Microphone() as source:
            self.get_logger().info("마이크를 통해 입력을 기다립니다...")
            try:
                # 마이크에서 소리 잡기
                audio = self.recognizer.listen(source, timeout=5)
                self.get_logger().info("음성을 처리 중입니다...")
                # 음성을 텍스트로 변환
                text = self.recognizer.recognize_google(audio, language='ko-KR')  # 한국어로 설정
                self.get_logger().info(f"들린 내용: {text}")

                # 방향 감지
                direction = self.mic_tuning.direction

                # 방향 데이터를 /detect_angle 토픽으로 발행
                angle_msg = Int32()
                angle_msg.data = direction
                self.angle_publisher.publish(angle_msg)

                # 변환된 텍스트가 '살려주세요'인 경우만 응답
                if "살려 주세요" in text:
                    self.get_logger().info("살려주세요 감지됨. 응답합니다.")
                    self.get_logger().info(f"소리가 오는 방향: {direction}도")
                    time.sleep(3)
                else:
                    self.get_logger().info("다른 음성입니다. 무시합니다.")
            except sr.UnknownValueError:
                self.get_logger().warn("음성을 이해할 수 없습니다.")
            except sr.RequestError as e:
                self.get_logger().error(f"음성 인식 서비스에 문제가 발생했습니다: {e}")
            except Exception as e:
                self.get_logger().error(f"오류가 발생했습니다: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ReSpeakerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
