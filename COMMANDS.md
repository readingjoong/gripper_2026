# 명령어 모음 (터미널에 칠 것)

순서대로 따라 치면 됨. 설명은 README.md, 새 컴퓨터 설치는 SETUP_NEW_UBUNTU.md.

터미널은 3개를 쓴다. VS Code에서 터미널 패널 오른쪽 위 `+` (새 터미널) 또는 분할 버튼 (Ctrl + Shift + 5).

| 터미널 | 용도 |
|---|---|
| 터미널 1 | 노드 실행 (계속 켜져 있음, 여기엔 다른 명령 치지 말 것) |
| 터미널 2 | 명령 보내기 (open, close, stop) |
| 터미널 3 | 상태 보기 (전류, 각도) |

---

## 매번 실행 전 (전원 켤 때마다)

1. 그리퍼 12V 전원 ON, U2D2 USB 연결

2. 포트 확인
   ```bash
   ls /dev/ttyUSB*
   ```
   `/dev/ttyUSB0` 이 나오면 OK

3. USB 지연 16 ms → 1 ms (재부팅하거나 USB 다시 꽂으면 16으로 돌아감)
   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   ```
   비밀번호 입력, `1` 이 나오면 OK

---

## 터미널 1: 노드 실행

```bash
cd ~/gripper
python3 gripper_node_v2.py --ros-args -p open_angle:=120.0 -p close_angle:=720.0 -p profile_vel:=70 -p profile_acc:=8
```

정상이면 마지막 줄:
```
gripper_node v2 준비 완료 | contact_detect=True, hold_mode=squeeze, torque_off_on_shutdown=False
```

- 켜면 토크가 들어가서 그리퍼가 그 자리에 고정됨 (스스로 열리거나 닫히지는 않음)
- 끄기: Ctrl + C. **v2는 꺼도 토크 유지** (쥔 물체 안 떨어뜨림). 힘을 완전히 빼려면 12V 전원 OFF

| 파라미터 | 값 | 뜻 |
|---|---|---|
| `open_angle` | 120.0 | 열림 위치 (0° 는 너무 벌어짐) |
| `close_angle` | 720.0 | 닫힘 위치 (손가락이 딱 맞닿음, 2026.10.29 실측) |
| `profile_vel` | 70 | 속도 (1 = 0.229 rpm → 70 = 약 96°/s) |
| `profile_acc` | 8 | 가속도 (클수록 빨리 최고 속도 도달) |

속도 70, 가속도 8 은 아직 빈손 5회 시험 전 (확인 필요). 50, 10 (기본값) 은 5회 통과.

---

## 터미널 2: 명령 보내기

```bash
ros2 service call /gripper/open std_srvs/srv/Trigger
```
열기. 응답 `success=True, message='open: arrived'`

```bash
ros2 service call /gripper/close std_srvs/srv/Trigger
```
닫기. 빈손이면 `close: arrived`, 물체를 잡으면 `close: contact | at ... deg, ... mA`
응답은 다 닫힐 때까지 안 옴 (몇 초 기다리는 게 정상)

```bash
ros2 service call /gripper/stop std_srvs/srv/Trigger
```
지금 위치에서 멈춤

팁: 위쪽 화살표(↑) 누르면 이전 명령이 다시 나옴

---

## 터미널 3: 상태 보기

```bash
ros2 topic echo /gripper/current_ma
```
모터 전류 (mA). 닫는 방향이 +, 여는 방향이 -

```bash
ros2 topic echo /gripper/contact
```
접촉 여부 (true / false)

```bash
ros2 topic echo /gripper/joint_states
```
모터 각도. `position` 값이 라디안이라 도(°) 로 보려면 × 57.3

```bash
ros2 topic echo /gripper/stroke_mm
```
손가락 간격 (mm). **지금 보정표가 틀려서 값 믿으면 안 됨** (600° 를 0 mm 로 계산하는데 실제로는 12.4 mm)

한 번만 보고 끝내려면 뒤에 `--once`. 계속 나오는 건 Ctrl + C 로 끔.

---

## 문제 생겼을 때

| 증상 | 확인할 것 |
|---|---|
| `포트를 열 수 없습니다` | U2D2 USB 연결, `ls /dev/ttyUSB*`, 다른 노드가 이미 켜져 있는지 |
| 숫자가 `ttyUSB1` 로 나옴 | 실행할 때 `-p port:=/dev/ttyUSB1` 추가, latency 명령도 `ttyUSB1` 로 |
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` 먼저 |
| service call 이 응답 없이 멈춤 | 터미널 1 노드가 켜져 있는지 |
| `다른 동작이 실행 중입니다` | 이전 open/close 가 아직 안 끝남. 기다리거나 stop |
| 빈손인데 중간에 `close: contact` | 오검출. `-p contact_threshold_ma:=95.0` 로 올려서 다시 실행 |
| 물체를 잡아도 `close: arrived` | 미검출. `-p contact_threshold_ma:=70.0` 로 내려서 다시 실행 |

노드 두 개(original, v2)를 동시에 켜지 말 것. 같은 포트를 잡으려고 충돌함.
