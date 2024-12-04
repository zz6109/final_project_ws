# 최종 프로젝트
- 붕괴 및 화재 건물내에서 구조 가능한 로봇
# 프로젝트 멤버
- 김재환, 장재희, 하정민
# 구현 할 주 기능
- 구글 어시스턴트 기반 의사소통 시스템
- 360도 음성 감지 및 추적
- 사물과 사람을 구분하여 인식
- 배터리 부족시 허브로 돌아가 자동충전
<hr/>

## 사고구조 로봇 하드웨어 제작
- 구현률 : 95%(누적)
- 24.11.25(오전), 하드웨어 30% 조립/김재환, 장재희, 하정민
  - 터틀봇 3대, Autocar jetson 분해 
- 24.11.25(오후), 하드웨어 30% 조립/김재환, 장재희, 하정민
  - 터틀봇 와플조립, 오픈 매니퓰레이터 조립
- 24.11.28(오전), 하드웨어 35% 조립/김재환
  - opencr 보드 설정, jetson os 업로드, 기타 전선 제작
<hr/>

## 하드웨어 브링업
- 구현률 : 25%(누적)
- 24.11.28(오후), 하드웨어 브링업 5% 작업, 김재환
  - ros-foxy 설치, turtlebot_lib설치, 센서관련 드라이버 설치  
- 24.11.29(오전), 하드웨어 브링업 5% 작업, 김재환
  - 브링업 관련 보드 및 dc모터 점검
- 24.11.29(오후), 하드웨어 브링업 10% 작업, 김재환
  - opencr 보드 펌웨어 점검, ros_domain 설정, teleop control 확인
- 24.12.02(오전) 하드웨어 브링업 5% 작업, 김재환
  - [ros2_control_node-2] [ERROR] [DynamixelSDKWrapper] [TxRxResult] There is no status packet!
  - 브링업중 위 에러 원인 찾기 
  - 해결책 1. 노트북에 도커를 설치해서 폭시로 버전 맞추어 브링업 X
  - 해결책 2. 터틀봇에 패키지를 핫픽스 버전으로 재설치 X
  - 해결책 3. 브링업 및 서보 관련 파일 전부 호환성 체크 X
  - 해결불가
- 24.12.02(오후) 하드웨어 브링업 2% 작업, 김재환
  - 기존 teleop코드가 tf의 위치를 기준으로 키보드의 입력을 받아 moveit으로 자동계산 되어 움직임이 산출됨, 조금만 움직이면 임계값에 도달하여 시스템이 죽음
  - moveit을 사용하지 않은 teleop 코드 작성
- 24.12.03(오전) 하드웨어 브링업 2% 작업, 김재환
  - moveit없이 파이썬으로 코드를 작성했지만 여전히 매니풀레이터 작동 불능
  - 해결책 1. 서비스 노드로 직접 값을 전달
  - 해결책 2. 하드웨어 브링업으로 생성되는 액션 노드를 활용해 값을 전달
- 24.12.03(오후) 하드웨어 브링업 3% 작업, 김재환
  - 해결잭 1. 시도: 조인트 각도를 조절할수 있는 서비스를 발행하지 않아서 직업 서비스 코드를 추가해야 하지만 액션서버가 활성화 되기때문에 스킵함
  - 해결책 2. 시도: 액션으로는 조정가능함을 확인
- 24.12.04(오전) 하드웨어 브링업 ??%, 김재환
  - 1. 3가지 포지션 값을 액션서버에 명시해놓고 서비스콜로 mode1 이라는 값이 들어오면 조인트가 움직이는 코드 작성(c++)
  - 2. 3가지 포지션 값을 액션서버에 명시해놓고 토픽 발행으로 mode1 이라는 값이 들어오면 조인트가 움직이는 코드 작성(python)
  - 두 방법의 효율 비교
<hr/> 

## 오픈소스 음성인식 API 기반 의사소통 시스템
- 구현률 : 0%
- 작업일자, 내용, 작업자(2024.11.00)/작업자-내용
<hr/>

## 360도 음성 감지 및 추적
- 구현률 : 0%
- 작업일자, 내용, 작업자(2024.11.00)/작업자-내용
<hr/>

## 사물과 사람을 구분하여 인식
- 구현률 : 10%(누적)
- 24.11.25(오전), 기본 소프트웨어 작업 5%, 하정민
  - ubuntu환경에서 realsense 정상 작동 확인
- 24.11.25(오후), 기본 소프트웨어 작업 5%, 하정민
  - realsense 기본 코드 테스트 및 호환성 점검
## 배터리 부족시 허브로 돌아가 자동충전
- 구현률 : 35%(누적)
- 24.11.25(오전), 하드웨어 구성 35%/김재환
  - 충전허브 3D 모델링, 실물 출력
- 24.11.25(오전), 하드웨어 구성 35%/김재환
  - 출력물 조립, 충전 전선 제작
<hr/>

