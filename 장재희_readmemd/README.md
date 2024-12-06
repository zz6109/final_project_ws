# 24.11.28

- 오전
1. Google Assistant API 사용 방법 학습

- 오후
1. ReSpeaker USB Mic Array 사용 방법 학습
2. doa_test.py 작성

# 24.11.29

- 오전
1. OpenAI Whisper 사용 방법 학습
-> 참고 : https://github.com/openai/whisper

1.1 pip install git+https://github.com/openai/whisper.git
-> OpenAI Whisper 저장소에서 최신 커밋과 해당 Python 종속성을 설치

2. OpenAI Whisper 설치
3. whisper_test.py 작성

- 오후
1. whisper_test.py 수정 및 테스트

-------------------------------------

* whisper_test.py 오류 발생 해결 과정

1. 'import sounddevice as sd' 오류 발생
-> PortAudio 라이브러리를 찾을 수 없어서 발생한 오류

2. sudo apt install portaudio19-dev 명령어 실행
-> PortAudio 설치

3. pip install --upgrade sounddevice
-> sounddevice 재설치

4. 오류 해결

# 24.12.03

- 오전
1. respeaker_turtlebot_test.py 작성
2. respeaker_turtlebot_test.py 테스트

- 오후
1. respeaker_turtlebot.py 작성

# 24.12.04

- 오전
1. respeaker_turtlebot.py 작성 및 수정

- 오후
1. respeaker_turtlebot.py 오류 수정
2. respeaker_turtlebot.py 테스트

# 24.12.05

-오전
1. Docker에 관해 학습

-오후
1. Docker에 관해 학습
2. YOLOv5를 기반으로 한 Docker 이미지를 실행 후 RealSense 설치

------------------------------

* YOLOv5를 기반으로 한 Docker 이미지를 실행 후 RealSense 설치

1. sudo docker pull ultralytics/yolov5:latest
2. sudo docker run --ipc=host -it ultralytics/yolov5:latest
3. apt-get update && apt-get install -y \
    git cmake build-essential libssl-dev \
    libusb-1.0-0-dev pkg-config
4. git clone https://github.com/IntelRealSense/librealsense.git
5. cd librealsense
6. mkdir build && cd build
7. apt-get update && apt-get install -y libx11-dev
&& apt-get update && apt-get install -y libxrandr-dev
&& apt-get update && apt-get install -y libxinerama-dev
&& apt-get update && apt-get install -y libxcursor-dev
&& apt-get update && apt-get install -y libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev
&& apt-get update && apt-get install -y libxi-dev libxext-dev

8. cmake ../ -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=bool:true
9. make -j$(nproc)
10. make install

-> 참고 : https://docs.ultralytics.com/ko/yolov5/environments/docker_image_quickstart_tutorial/#prerequisites

# 24.12.06

- 오전
1. YOLOv5를 기반으로 한 Docker 이미지를 실행 후 RealSense 설치

- 오후
1. YOLOv5를 기반으로 한 Docker 이미지를 실행 후 RealSense 설치
2. realsense_test.py 작성