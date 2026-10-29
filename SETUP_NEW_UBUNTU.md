# 새 우분투 컴퓨터에서 처음부터 실행하기

아무것도 안 깔린 컴퓨터에서 그리퍼 노드를 돌리기까지의 전체 과정.
한 번만 하면 되는 것(1~6)과 매번 하는 것(7)으로 나뉨.
명령어는 한 줄씩 복사해서 터미널에 붙여넣고 Enter. 결과를 보고 다음 줄로.

**준비물**: Ubuntu 24.04 컴퓨터, 인터넷, 그리퍼 + 12V 전원 + U2D2 + USB 케이블

---

## 0. 기본 개념 (처음이면 읽기)

- **터미널**: 글자로 명령을 치는 창. 우분투에서 `Ctrl + Alt + T`
- **`sudo`**: 관리자 권한으로 실행. 시스템에 프로그램을 깔거나 설정을 바꿀 때 필요. 처음 한 번 비밀번호를 물어봄 (입력해도 화면에 안 보이는 게 정상)
- **`apt`**: 우분투 프로그램 설치기. 앱스토어의 터미널 버전
- **ROS2**: 로봇 프로그램끼리 통신하게 해주는 도구 모음. 우리는 **Jazzy** 버전 (Ubuntu 24.04 짝꿍)
- **`~`**: 내 홈 폴더 (`/home/내이름`)
- 붙여넣기는 `Ctrl + V` 가 아니라 **`Ctrl + Shift + V`** (터미널에서는)
- 명령이 끝나면 다시 `내이름@컴퓨터:~$` 같은 줄이 나옴. 그 전에는 기다리기

---

## 1. 우분투 버전 확인

```bash
lsb_release -a
```
- `Release: 24.04` 가 나와야 함
- 22.04 면 ROS2 Jazzy 가 안 깔림 (22.04 는 Humble). 이 경우 멈추고 물어보기

---

## 2. ROS2 Jazzy 설치 (20~40분)

공식 설명서: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
아래가 안 되면 공식 설명서를 따를 것 (설치 방법이 가끔 바뀜).

이미 깔려 있는지 먼저 확인:
```bash
ls /opt/ros
```
`jazzy` 가 나오면 **2번 건너뛰고 3번으로.** `No such file or directory` 면 아래 진행.

**2-1. 추가 프로그램 저장소 켜기**
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
```
- `software-properties-common`: 저장소를 추가하는 도구
- `curl`: 인터넷에서 파일을 받는 도구
- `universe`: 우분투의 커뮤니티 프로그램 저장소. ROS2 가 이걸 필요로 함

**2-2. ROS2 저장소 등록**
```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
- 첫 줄: ROS 저장소 설정 파일의 최신 버전 번호를 알아옴
- 둘째 줄: 그 설정 파일(.deb)을 내려받음
- 셋째 줄: 설치. 이제 `apt` 가 ROS 프로그램을 찾을 수 있게 됨

**2-3. ROS2 설치**
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop
```
- `apt update`: 프로그램 목록 새로 고침
- `apt upgrade`: 이미 깔린 프로그램 업데이트
- `ros-jazzy-desktop`: ROS2 본체 + 시각화 도구. 가장 오래 걸림

**2-4. 터미널 열 때마다 ROS 자동으로 불러오기**
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- `~/.bashrc`: 터미널을 열 때마다 자동으로 실행되는 설정 파일
- `>>`: 파일 끝에 한 줄 추가 (`>` 하나면 파일을 덮어써서 날아가니 주의)
- 이걸 안 하면 터미널마다 `source /opt/ros/jazzy/setup.bash` 를 직접 쳐야 함
- **이 명령은 한 번만.** 여러 번 치면 같은 줄이 중복으로 들어감

확인:
```bash
ros2 --help
```
사용법이 쭉 나오면 OK. `command not found` 면 터미널을 껐다 켜보기

---

## 3. 다이나믹셀 라이브러리 설치

```bash
sudo apt install -y ros-jazzy-dynamixel-sdk python3-numpy
```
- `ros-jazzy-dynamixel-sdk`: 다이나믹셀 모터와 통신하는 라이브러리 (2026.10.29 이 이름으로 설치되는 것 확인함)
- `python3-numpy`: 수치 계산 라이브러리

확인:
```bash
python3 -c "import dynamixel_sdk; print('OK')"
```
`OK` 가 나오면 성공. `No module named 'dynamixel_sdk'` 면 2-4 의 source 가 안 된 것. 터미널 새로 열고 다시

---

## 4. USB 포트 권한 주기

```bash
sudo usermod -aG dialout $USER
```
- 리눅스는 USB 시리얼 포트(`/dev/ttyUSB0`)를 `dialout` 그룹만 쓸 수 있게 막아둠
- `-aG dialout`: 내 계정을 그 그룹에 추가 (`-a` 를 빼먹으면 다른 그룹에서 빠지니 꼭 붙이기)
- `$USER`: 지금 로그인한 내 계정 이름

**로그아웃 후 다시 로그인** (또는 재부팅) 해야 적용됨.

확인:
```bash
groups
```
목록에 `dialout` 이 있으면 OK

---

## 5. (선택) USB 지연 자동 설정

USB 지연을 1 ms 로 바꾸는 걸 매번 안 해도 되게 만드는 방법. 안 해도 7번에서 매번 손으로 하면 됨.

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVERS=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules
sudo udevadm control --reload-rules
```
- `/etc/udev/rules.d/`: USB 장치가 꽂힐 때 자동으로 실행할 규칙을 넣는 폴더
- 규칙 내용: FTDI 칩(U2D2 안에 들어있음) 시리얼 장치가 꽂히면 latency_timer 를 1 로
- 두 번째 줄: 규칙 다시 읽기
- 적용하려면 U2D2 를 뽑았다 다시 꽂기

확인 (U2D2 꽂은 상태에서):
```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
`1` 이면 성공. 이 컴퓨터에서는 아직 안 해봄 (확인 필요)

---

## 6. 그리퍼 파일 가져오기

**방법 A: zip 파일로**
```bash
unzip ~/Downloads/gripper.zip -d ~
```
- `~/Downloads/gripper.zip` 을 홈 폴더에 풀어서 `~/gripper/` 가 생김

**방법 B: GitHub 저장소로** (저장소를 만든 경우)
```bash
sudo apt install -y git
git clone <저장소 주소> ~/gripper
```
- `git clone`: 저장소를 통째로 내려받기

확인:
```bash
ls ~/gripper
```
`gripper_node_v2.py`, `README.md` 등이 보이면 OK

**(선택) VS Code 설치**
```bash
sudo snap install code --classic
code ~/gripper
```

---

## 7. 실행 (매번)

여기부터는 COMMANDS.md 와 같음.

1. 그리퍼 12V 전원 ON, U2D2 USB 연결
2. 포트 확인: `ls /dev/ttyUSB*` → `/dev/ttyUSB0`
3. USB 지연 (5번 안 했으면): `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` → `1`
4. 터미널 1 에서 노드 실행
   ```bash
   cd ~/gripper
   python3 gripper_node_v2.py --ros-args -p open_angle:=120.0 -p close_angle:=720.0 -p profile_vel:=70 -p profile_acc:=8
   ```
   `gripper_node v2 준비 완료` 가 나오면 OK
5. 터미널 2 에서 명령
   ```bash
   ros2 service call /gripper/open std_srvs/srv/Trigger
   ros2 service call /gripper/close std_srvs/srv/Trigger
   ```

**처음 켤 때 주의**: 다이나믹셀은 전원을 껐다 켜면 각도를 0~360° 사이로 다시 셀 수 있음. 그리퍼는 120~720° 를 쓰기 때문에, 처음 open 을 부르기 전에 각도를 확인할 것:
```bash
ros2 topic echo --once /gripper/joint_states
```
`position` × 57.3 = 각도(°). 그리퍼가 거의 열려 있는데 각도가 큰 값(600 근처)이거나, 거의 닫혀 있는데 작은 값이면 기준이 어긋난 것. **그 상태로 open/close 부르지 말고 멈춰서 확인** (확인 필요)

---

## 막혔을 때

| 증상 | 원인과 해결 |
|---|---|
| `Unable to locate package ros-jazzy-...` | 2-2 저장소 등록이 안 됨. 2-2 다시 하고 `sudo apt update` |
| `ros2: command not found` | 2-4 안 됨. `source /opt/ros/jazzy/setup.bash` |
| `No module named 'dynamixel_sdk'` | 3번 설치 안 됨, 또는 source 안 함 |
| `포트를 열 수 없습니다` / `Permission denied` | 4번 후 로그아웃 안 함. `groups` 에 dialout 있는지 |
| `/dev/ttyUSB*` 아무것도 안 나옴 | USB 케이블, U2D2 LED 확인. `lsusb` 에 `Future Technology Devices` 가 보여야 함 |
| 노드는 켜지는데 모터가 반응 없음 | 12V 전원, 모터 ID(3), baudrate(4000000) 가 Wizard 설정과 같은지 |
