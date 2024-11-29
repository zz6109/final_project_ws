# 2024.11.29
# OpenAI Whisper를 사용하여 음성이 감지된 각도(DOA 값)와 텍스트 출력

from tuning import Tuning 
import usb.core
import usb.util
import sounddevice as sd
import whisper
import numpy as np
import time

model = whisper.load_model("base")

# 오디오 스트림 설정
SAMPLE_RATE = 16000  # Whisper 권장 샘플링 속도
CHANNELS = 1  # Mono Audio
DEVICE_ID = None  # ReSpeaker USB Mic Array의 입력 장치 ID
audio_buffer = []  # 오디오 데이터를 저장하는 버퍼


def find_respeaker_device():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev:
        print("ReSpeaker USB Mic Array detected!")
        return Tuning(dev)
    else:
        print("ReSpeaker USB Mic Array not found. Please check the connection.")
        return None

# 오디오 데이터를 처리하는 콜백 함수
def audio_callback(indata, frames, time, status):
    global audio_buffer
    if status:
        print(f"Audio callback status: {status}")
    audio_buffer.extend(indata[:, 0])  # 첫 번째 채널 데이터 추가 (Mono)

# 버퍼에 쌓인 오디오 데이터를 Whisper로 변환
def recognize_audio():
    global audio_buffer
    if len(audio_buffer) >= SAMPLE_RATE * 5:  # 5초 분량 데이터 처리
        audio_data = np.array(audio_buffer[:SAMPLE_RATE * 5])
        audio_buffer = audio_buffer[SAMPLE_RATE * 5:]  # 처리된 데이터 제거

        print("Recognizing speech...")
        # 언어를 한국어로 고정
        result = model.transcribe(audio_data, fp16=False, language="ko")
        return result["text"]
    return None


def main():
    global DEVICE_ID
    print("Available audio devices:")
    print(sd.query_devices())
    DEVICE_ID = int(input("Enter the device ID for ReSpeaker USB Mic Array: "))

    # ReSpeaker Mic Array 초기화
    respeaker = find_respeaker_device()
    if not respeaker:
        return

    print("Starting voice detection with DOA (Direction of Arrival)...")

    try:
        # 오디오 스트림 시작
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            device=DEVICE_ID,
            callback=audio_callback,
        ):
            while True:
                # DOA 및 음성 감지
                is_voice = respeaker.is_voice()
                if is_voice:
                    doa = respeaker.direction  # DOA 값을 읽음
                    print(f"Voice detected! Direction: {doa}°")

                    # Whisper로 음성 텍스트 변환
                    text = recognize_audio()
                    if text:
                        print(f"Recognized Text: {text}")
                else:
                    print("No voice detected.")

                time.sleep(1)  # 1초 간격으로 DOA 갱신
    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting.")


if __name__ == "__main__":
    main()
