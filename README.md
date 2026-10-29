# 그리퍼 ROS2 노드 (학부학술제용)

2지 그리퍼(Dynamixel XM540, 리드스크류 구동)를 ROS2 Jazzy 노드로 돌리는 폴더.
나중에 Doosan M1013 + MoveIt 과 같이 써서 pick and place 시연을 하는 게 목표.

## 폴더 구성

| 파일 | 설명 |
|---|---|
| `gripper_node_v2.py` | **이걸 실행.** 접촉 감지와 파지 유지 방식을 고친 버전 |
| `gripper_node_original.py` | 최지원이 만든 원본 (비교용, 수정하지 않음) |
| `README.md` | 이 파일. 사람이 읽는 설명서 |
| `CLAUDE.md` | Claude Code가 폴더를 열면 자동으로 읽는 프로젝트 설명 |

## 0. 폴더 열기 (VS Code)

```bash
unzip ~/Downloads/gripper.zip -d ~
code ~/gripper
```
- `unzip ... -d ~`: zip을 홈 폴더에 푼다. 그러면 `~/gripper/` 폴더 안에 파일들이 생긴다
- `code ~/gripper`: VS Code로 이 폴더를 연다

VS Code 안에서 터미널 열기: 상단 메뉴 Terminal > New Terminal (단축키 Ctrl + `)
Claude Code 시작: 그 터미널에서 `claude` 입력

## 1. 처음 한 번만 하는 설치

```bash
ls /opt/ros
```
`jazzy` 가 나와야 함. 안 나오면 ROS2 Jazzy부터 설치.

```bash
sudo apt install -y ros-jazzy-dynamixel-sdk python3-numpy
python3 -c "import dynamixel_sdk; print('OK')"
```
`OK` 가 나오면 성공. 첫 줄이 "Unable to locate package" 로 실패하면 대신:
```bash
pip install dynamixel-sdk --break-system-packages
```
(패키지 이름 확인 필요. 둘 중 되는 쪽 사용)

```bash
sudo usermod -aG dialout $USER
```
USB 시리얼 포트를 쓸 권한을 준다. **실행 후 로그아웃했다가 다시 로그인해야 적용됨.**
확인: `groups` 결과에 `dialout` 이 있으면 OK.

## 2. 매번 실행 전 확인

1. 그리퍼 12V 전원 ON, U2D2를 USB로 연결
2. 포트 확인
   ```bash
   ls /dev/ttyUSB*
   ```
   `/dev/ttyUSB0` 이 나와야 함. 숫자가 다르면 실행할 때 `-p port:=/dev/ttyUSB1` 처럼 지정
3. USB 지연을 16 ms에서 1 ms로 (재부팅하면 원래대로 돌아가서 매번 필요)
   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   ```
   `1` 이 출력되면 성공

## 3. 실행

**터미널 1: 노드 실행**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/gripper
python3 gripper_node_v2.py
```
정상이면 마지막에 이런 줄이 나옴:
```
gripper_node v2 준비 완료 | contact_detect=True, hold_mode=squeeze, torque_off_on_shutdown=False
```
끄기: Ctrl + C

**터미널 2: 명령 보내기** (`source /opt/ros/jazzy/setup.bash` 먼저)
```bash
ros2 service call /gripper/open std_srvs/srv/Trigger
ros2 service call /gripper/close std_srvs/srv/Trigger
ros2 service call /gripper/stop std_srvs/srv/Trigger
```
응답에 `success=True` 가 나오면 성공. close는 접촉하면 `close: contact | at ... deg, stroke ... mm, ... mA` 형태로 접촉 순간 값을 알려줌.

**터미널 3: 상태 보기**
```bash
ros2 topic echo /gripper/current_ma
ros2 topic echo /gripper/contact
ros2 topic echo /gripper/stroke_mm
```
값이 실시간으로 올라옴. Ctrl + C로 끔.

## 4. 오늘 확인할 것 (테스트 체크리스트)

| 번호 | 시험 | 기대 결과 | 결과 |
|---|---|---|---|
| 1 | 빈손으로 open | `open: arrived` | |
| 2 | 빈손으로 close (5회) | 매번 `close: arrived`, 중간에 멈추지 않음 | |
| 3 | 물체 쥐어주고 close (5회) | `close: contact`, 접촉 각도와 전류 기록 | |
| 4 | 잡은 상태에서 물체를 손으로 당김 | 버팀 (squeeze 동작) | |
| 5 | 잡은 상태에서 노드 Ctrl + C | 물체를 떨어뜨리지 않음 (토크 유지) | |

**조정 방법**
- 2번에서 중간에 멈춤 (오검출) → 기준을 올림: `-p contact_threshold_ma:=95.0`
- 3번에서 감지 못함 (미검출) → 기준을 내림: `-p contact_threshold_ma:=70.0`
- 4번에서 버티지 못함 → 힘 상한을 올림: `-p goal_current:=60` (161 mA, PPT 실험 조건)
- 원본처럼 접촉 시 그 자리 정지 → `-p hold_mode:=stop`

여러 개 같이 쓸 때:
```bash
python3 gripper_node_v2.py --ros-args -p contact_threshold_ma:=95.0 -p goal_current:=60
```

## 5. 주요 파라미터

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | U2D2 포트 |
| `goal_current` | 40 (107.6 mA) | 전류 상한 = 쥐는 힘 상한 |
| `contact_detect` | true | 접촉 감지 켜기 |
| `contact_threshold_ma` | 80.0 | 이 전류를 넘고 위치가 멈추면 접촉 |
| `contact_ignore_s` | 0.5 | 출발 후 이 시간 동안은 감지 안 함 (시작 마찰 spike 대비) |
| `stall_window_s`, `stall_tol_deg` | 0.2 s, 2.0° | 0.2 s 동안 2° 미만 움직이면 "멈춤" |
| `hold_mode` | squeeze | 접촉 후 계속 쥐기 (`stop` 이면 그 자리 정지) |
| `squeeze_offset_deg` | 30.0 | squeeze 때 현재 위치보다 더 닫으라고 주는 각도 |
| `torque_off_on_shutdown` | false | 노드 종료 시 토크 끌지 |
| `open_angle`, `close_angle` | 0, 600° | 완전 열림, 완전 닫힘 모터 각도 |

## 6. 원본 대비 바뀐 점

1. 접촉 감지 기본 ON
2. 출발 후 0.5 s 동안 감지 무시 (PPT: 시작 정지마찰 spike 약 78~100 mA)
3. 접촉 = 전류 기준 초과 + 위치 멈춤 (PPT: 200~340° 구간은 전류만 높고 계속 움직임)
4. 접촉 후 squeeze로 계속 쥠 (원본은 그 자리 정지 → 쥐는 힘이 거의 0이 될 수 있음)
5. 종료 시 토크 유지 기본
6. `/gripper/contact` 토픽 추가, close 응답에 접촉 순간 값 포함

## 7. 아직 확인 안 된 것

- 실제 모터로 v2를 돌려본 적 없음. 접촉 판정 로직만 가짜 데이터로 시험함
- 보정표(cal_angle, cal_stroke)가 랩미팅 PPT 표와 일부 다름 (0 대 8, 84 대 84.5, 57 대 57.5, 9 대 9.5). 어느 쪽이 맞는지 확인 필요
- `ros-jazzy-dynamixel-sdk` 패키지 이름 확인 필요
- 로봇 PC에서 M1013 MoveIt과 동시에 돌리는 것은 아직 안 해봄

## 8. 다음 단계

1. 위 체크리스트 1~5 통과
2. 로봇 PC에서 M1013 (두산 드라이버 + MoveIt + 조교님 pan GUI) 과 동시에 실행, GUI로 팔 이동 후 `/gripper/close` 호출 (손으로 하는 통합)
3. pick 스크립트: 접근 → 하강 → close → 상승 → 이동 → open 을 한 파일로
4. 힘 추정: 접촉 후 squeeze 전류 → 모터 토크 → 자코비안 → 손끝 힘, 로드셀과 비교
