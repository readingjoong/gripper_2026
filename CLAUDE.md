# 프로젝트 안내 (Claude Code용)

## 무엇
성균관대 김규남 교수님 연구실 학부연구생 프로젝트. 2지 그리퍼(Dynamixel XM540, 리드스크류 구동)를
ROS2 Jazzy 노드로 제어하고, 나중에 Doosan M1013 + MoveIt2 와 통합해 pick and place 시연.
최종 목표: 모터 전류로 손끝 힘을 추정(자코비안, τ = JᵀF)하고 로드셀 값과 비교. 학부학술제 11/17.

## 사용자
기계공학부 3학년. 터미널, ROS, 코딩은 처음 배우는 중.
- 명령어를 제안할 때는 각 부분이 무슨 뜻인지, 정상이면 무엇이 출력되는지 쉽게 설명
- 설명은 한국어 반말, 짧고 직접적으로

## 환경
- Ubuntu 24.04, ROS2 Jazzy, Python 3.12 (`/usr/bin/python3`, conda 쓰지 않음)
- 실행: `source /opt/ros/jazzy/setup.bash` 후 `python3 gripper_node_v2.py`
- U2D2 포트 `/dev/ttyUSB0`, baudrate 4000000, DXL ID 3, Protocol 2.0
- 모드 5 (Current-based Position), 전류 단위 1 raw = 2.69 mA, 위치 4096 pulse/rev

## 파일
- `gripper_node_v2.py`: 작업 대상
- `gripper_node_original.py`: 최지원 원본. **수정하지 말 것** (비교용)
- `README.md`: 설치, 실행, 테스트 체크리스트

## 규칙
1. **하드웨어를 움직이는 명령은 실행 전에 반드시 사용자에게 무엇이 움직이는지 말하고 확인받을 것.**
   해당: `python3 gripper_node*.py`, `ros2 service call /gripper/*`, `ros2 topic pub /gripper/*`,
   그리고 로봇팔 관련 모든 명령 (`ros2 launch dsr_bringup2 ...`, `pan_m1013_control`, `--execute`,
   MoveIt execute, `/dsr*` 서비스). 로봇팔은 사람이 다칠 수 있으니 절대 임의로 실행하지 말 것.
2. 파라미터로 해결되는 건 코드 수정보다 `--ros-args -p 이름:=값` 을 먼저 제안
3. 코드를 고치면 무엇을 왜 바꿨는지 짧게 설명하고, README의 해당 부분도 같이 갱신
4. 확인 안 된 내용은 "확인 필요"로 표시. 동작 결과를 부풀리지 말 것
5. `sudo` 가 필요한 명령은 이유를 먼저 설명
6. 문서에 가운뎃점(·) 쓰지 말 것

## 알려진 데이터 (2026.10.30 랩미팅 PPT, 무부하 10회)
- 닫기 90 → 0 mm, 약 7.0 s, 정속 약 13.1 mm/s
- 이동 중 평균 전류: close 약 +33 mA, open 약 -42 mA
- 시작 정지마찰 spike: 약 78~100 mA
- 200~340° (stroke 약 40~60 mm) 구간에서 양방향 마찰 증가
- 위치별 threshold (평균 + 3σ): close 52~136 mA, open 57~149 mA
- 보정표가 코드와 PPT에서 일부 다름 (확인 필요)

## 다음 할 일
1. README 체크리스트 1~5 시험 후 contact_threshold_ma, goal_current 조정
2. 로봇 PC에서 M1013 MoveIt (조교님 `pan_m1013_control`) 과 동시 실행
3. pick 스크립트 (MoveIt `/move_action` + `/gripper/*` 서비스)
4. 힘 추정 코드
