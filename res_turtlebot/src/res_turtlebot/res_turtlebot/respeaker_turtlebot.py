from .tuning import Tuning
import usb.core
import usb.util
import sounddevice as sd
import whisper
import numpy as np
import time
import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import Twist 

# Whisper 모델 로드
model = whisper.load_model("medium")

# 오디오 스트림 설정
SAMPLE_RATE = 16000  
CHANNELS = 1  
DEVICE_ID = None 
audio_buffer = []  

# ROS2 노드 초기화 및 퍼블리셔 생성
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

def find_respeaker_device():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev:
        print("ReSpeaker USB Mic Array detected!")
        return Tuning(dev)
    else:
        print("ReSpeaker USB Mic Array not found. Please check the connection.")
        return None

def audio_callback(indata, frames, time, status):
    global audio_buffer
    if status:
        print(f"Audio callback status: {status}")
    audio_buffer.extend(indata[:, 0]) 

def recognize_audio():
    global audio_buffer
    if len(audio_buffer) >= SAMPLE_RATE * 5:  
        audio_data = np.array(audio_buffer[:SAMPLE_RATE * 5])
        audio_buffer = audio_buffer[SAMPLE_RATE * 5:]  

        print("Recognizing speech...")
        result = model.transcribe(audio_data, fp16=False, language="ko")
        return result["text"]
    return None

def main():
    global DEVICE_ID
    print("Available audio devices:")
    print(sd.query_devices())
    DEVICE_ID = int(input("Enter the device ID for ReSpeaker USB Mic Array: "))

    respeaker = find_respeaker_device()
    if not respeaker:
        return

    rclpy.init()
    turtlebot_control = TurtleBotVoiceControl()

    print("Starting voice detection with DOA (Direction of Arrival)...")

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            device=DEVICE_ID,
            callback=audio_callback,
        ):
            while rclpy.ok():
                is_voice = respeaker.is_voice()
                if is_voice:
                    doa = respeaker.direction 
                    print(f"Voice detected! Direction: {doa}°")

                    turtlebot_control.send_rotation_command(doa)

                    text = recognize_audio()
                    if text:
                        print(f"Recognized Text: {text}")
                else:
                    print("No voice detected.")

                time.sleep(1) 
    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()