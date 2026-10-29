#!/usr/bin/env python3
"""
gripper_node_v2.py  (ROS2 Jazzy, rclpy + dynamixel_sdk)

최지원 gripper_node.py 를 바탕으로 접촉 감지와 파지 유지 방식만 바꾼 버전.
원본 gripper_node.py 는 수정하지 않았음.

v2 변경점
  1. 접촉 감지 기본값 켜짐 (contact_detect=True)
  2. 출발 직후 contact_ignore_s(기본 0.5 s) 동안은 감지 안 함
     -> PPT 무부하 실험의 시작 정지마찰 spike(약 78~100 mA) 오검출 방지
  3. 접촉 판정 = "전류가 기준 이상" AND "위치가 거의 안 움직임(stall)"
     -> 200~340 deg 마찰 구간에서 전류만 높고 계속 움직이는 경우는 접촉 아님
  4. 접촉 후 동작 hold_mode
     'squeeze'(기본): 현재 위치보다 squeeze_offset_deg 만큼 더 닫는 목표를 줌.
                      Current-based Position 모드라 Goal Current 로 힘이 제한된 채 계속 쥠.
                      힘 추정 실험에 필요한 "쥐고 있는 동안의 전류"가 남음.
     'stop'         : 원본과 같이 현재 위치에서 정지(쥐는 힘이 거의 0 으로 떨어질 수 있음).
  5. 종료 시 토크 유지가 기본 (torque_off_on_shutdown=False)
     -> 로봇팔에 달고 물체를 든 채 노드가 꺼져도 힘이 바로 빠지지 않게
  6. /gripper/contact (std_msgs/Bool) 토픽 추가, close 응답에 접촉 순간 각도/전류 포함

Services (std_srvs/Trigger)
  /gripper/open, /gripper/close, /gripper/stop, /gripper/run_test
Subscribers
  /gripper/goal_stroke  (std_msgs/Float32)  목표 stroke [mm]
Publishers
  /gripper/stroke_mm, /gripper/current_ma (Float32), /gripper/contact (Bool),
  /gripper/joint_states (JointState, position = 모터 각도 [rad])

실행 예
  source /opt/ros/jazzy/setup.bash
  python3 gripper_node_v2.py
  python3 gripper_node_v2.py --ros-args -p contact_threshold_ma:=70.0 -p hold_mode:=stop
"""
import csv
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# ================== XM540 Control Table ==================
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132

CURRENT_UNIT_MA = 2.69          # XM540: 1 raw = 2.69 mA
OP_MODE_CURRENT_BASED_POS = 5   # Current-based Position Control (multi-turn)
DEG_PER_PULSE = 360.0 / 4096.0


class ContactDetector:
    """ROS 와 무관한 순수 판정 로직 (단독 테스트 가능).

    update(t, angle_deg, current_ma) 를 매 샘플 호출하면 접촉이면 True.
    접촉 조건 (전부 만족):
      a) 동작 시작 후 ignore_s 이상 지남
      b) |current| > threshold_ma 가 n_samples 회 연속
      c) 최근 stall_window_s 동안 각도 변화폭 < stall_tol_deg
    """

    def __init__(self, threshold_ma, n_samples, ignore_s, stall_window_s, stall_tol_deg):
        self.threshold_ma = threshold_ma
        self.n_samples = n_samples
        self.ignore_s = ignore_s
        self.stall_window_s = stall_window_s
        self.stall_tol_deg = stall_tol_deg
        self.reset(0.0)

    def reset(self, t0):
        self.t0 = t0
        self.over = 0
        self.hist = deque()   # (t, angle)

    def update(self, t, angle_deg, current_ma):
        self.hist.append((t, angle_deg))
        while self.hist and t - self.hist[0][0] > self.stall_window_s:
            self.hist.popleft()

        if t - self.t0 < self.ignore_s:
            self.over = 0
            return False

        self.over = self.over + 1 if abs(current_ma) > self.threshold_ma else 0
        if self.over < self.n_samples:
            return False

        # stall 판정에는 창 길이만큼의 기록이 쌓여 있어야 함
        if t - self.hist[0][0] < 0.8 * self.stall_window_s:
            return False
        angles = [a for _, a in self.hist]
        return (max(angles) - min(angles)) < self.stall_tol_deg


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # ---------- parameters ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4000000)
        self.declare_parameter('dxl_id', 3)

        self.declare_parameter('goal_current', 40)       # raw (40 x 2.69 = 107.6 mA)
        self.declare_parameter('profile_acc', 10)
        self.declare_parameter('profile_vel', 50)

        self.declare_parameter('open_angle', 0.0)        # deg
        self.declare_parameter('close_angle', 600.0)     # deg
        self.declare_parameter('arrive_tol', 20)         # raw pulse
        self.declare_parameter('move_timeout', 30.0)     # s
        self.declare_parameter('dwell_time', 1.0)        # s (run_test)
        self.declare_parameter('sample_dt', 0.01)        # s

        # 접촉 감지 (v2)
        self.declare_parameter('contact_detect', True)
        self.declare_parameter('contact_threshold_ma', 80.0)  # goal_current(107.6 mA)보다 낮아야 함
        self.declare_parameter('contact_samples', 3)
        self.declare_parameter('contact_ignore_s', 0.5)
        self.declare_parameter('stall_window_s', 0.2)
        self.declare_parameter('stall_tol_deg', 2.0)
        self.declare_parameter('hold_mode', 'squeeze')        # 'squeeze' | 'stop'
        self.declare_parameter('squeeze_offset_deg', 30.0)

        # angle -> stroke 보정표 (각도 오름차순)
        # 주의: 2026.10.30 랩미팅 PPT 표(0, 84, 57, 9 등)와 일부 값이 다름. 어느 쪽이 맞는지 확인 필요.
        self.declare_parameter('cal_angle',
                               [8.0, 50.0, 104.0, 160.0, 232.0, 292.0,
                                373.0, 436.0, 488.0, 531.0, 600.0])
        self.declare_parameter('cal_stroke',
                               [90.0, 84.5, 77.0, 68.0, 57.5, 48.0,
                                35.0, 24.0, 16.0, 9.5, 0.0])

        self.declare_parameter('publish_rate', 10.0)     # Hz
        self.declare_parameter('csv_dir', str(Path.home() / 'gripper_logs'))
        self.declare_parameter('torque_off_on_shutdown', False)

        p = self.get_parameter
        self.port_name = p('port').value
        self.baudrate = p('baudrate').value
        self.dxl_id = p('dxl_id').value
        self.open_angle = p('open_angle').value
        self.close_angle = p('close_angle').value
        self.arrive_tol = p('arrive_tol').value
        self.move_timeout = p('move_timeout').value
        self.dwell_time = p('dwell_time').value
        self.sample_dt = p('sample_dt').value
        self.cal_angle = np.array(p('cal_angle').value, dtype=float)
        self.cal_stroke = np.array(p('cal_stroke').value, dtype=float)
        if len(self.cal_angle) != len(self.cal_stroke) or len(self.cal_angle) < 2:
            raise ValueError('cal_angle / cal_stroke 길이가 같아야 하고 2개 이상이어야 합니다.')

        goal_current_ma = p('goal_current').value * CURRENT_UNIT_MA
        if p('contact_threshold_ma').value >= goal_current_ma:
            self.get_logger().warn(
                f'contact_threshold_ma({p("contact_threshold_ma").value:.0f} mA)가 '
                f'goal current({goal_current_ma:.0f} mA) 이상입니다. 전류가 상한에 막혀 '
                '접촉을 감지하지 못할 수 있습니다.')

        # ---------- state / locks ----------
        self._io_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._stop_flag = threading.Event()
        self._last_state = (0, 0.0, 0.0)       # (raw position, angle deg, current mA)
        self._contact = False
        self._contact_info = None              # (angle deg, current mA) at contact

        # ---------- dynamixel ----------
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f'포트를 열 수 없습니다: {self.port_name} '
                               '(U2D2 연결, dialout 권한 확인)')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'Baudrate 설정 실패: {self.baudrate}')
        self.get_logger().info(f'포트 연결 성공: {self.port_name} @ {self.baudrate}')
        self._init_motor(p('goal_current').value, p('profile_acc').value, p('profile_vel').value)

        # ---------- ROS interfaces ----------
        cb = ReentrantCallbackGroup()
        self.pub_stroke = self.create_publisher(Float32, 'gripper/stroke_mm', 10)
        self.pub_current = self.create_publisher(Float32, 'gripper/current_ma', 10)
        self.pub_contact = self.create_publisher(Bool, 'gripper/contact', 10)
        self.pub_js = self.create_publisher(JointState, 'gripper/joint_states', 10)

        self.create_service(Trigger, 'gripper/open', self._srv_open, callback_group=cb)
        self.create_service(Trigger, 'gripper/close', self._srv_close, callback_group=cb)
        self.create_service(Trigger, 'gripper/stop', self._srv_stop, callback_group=cb)
        self.create_service(Trigger, 'gripper/run_test', self._srv_run_test, callback_group=cb)
        self.create_subscription(Float32, 'gripper/goal_stroke',
                                 self._on_goal_stroke, 10, callback_group=cb)
        self.create_timer(1.0 / p('publish_rate').value, self._publish_state, callback_group=cb)

        self.get_logger().info(
            f'gripper_node v2 준비 완료 | contact_detect={p("contact_detect").value}, '
            f'hold_mode={p("hold_mode").value}, '
            f'torque_off_on_shutdown={p("torque_off_on_shutdown").value}')

    # ================== low level ==================
    def _init_motor(self, goal_current, acc, vel):
        with self._io_lock:
            pk, pt, i = self.packet, self.port, self.dxl_id
            pk.write1ByteTxRx(pt, i, ADDR_TORQUE_ENABLE, 0)
            pk.write1ByteTxRx(pt, i, ADDR_OPERATING_MODE, OP_MODE_CURRENT_BASED_POS)
            pk.write1ByteTxRx(pt, i, ADDR_TORQUE_ENABLE, 1)
            pk.write2ByteTxRx(pt, i, ADDR_GOAL_CURRENT, int(goal_current))
            pk.write4ByteTxRx(pt, i, ADDR_PROFILE_ACCELERATION, int(acc))
            pk.write4ByteTxRx(pt, i, ADDR_PROFILE_VELOCITY, int(vel))
        self.get_logger().info(
            f'모터 초기화: mode=5, goal_current={goal_current} '
            f'({goal_current * CURRENT_UNIT_MA:.1f} mA), acc={acc}, vel={vel}')

    def _read_state(self):
        """(raw_position, angle_deg, current_mA). 통신 실패 시 직전 값을 반환."""
        with self._io_lock:
            pos, r1, _ = self.packet.read4ByteTxRx(self.port, self.dxl_id, ADDR_PRESENT_POSITION)
            cur, r2, _ = self.packet.read2ByteTxRx(self.port, self.dxl_id, ADDR_PRESENT_CURRENT)
        if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
            return self._last_state
        if pos >= 2 ** 31:
            pos -= 2 ** 32
        if cur >= 32768:
            cur -= 65536
        self._last_state = (pos, pos * DEG_PER_PULSE, cur * CURRENT_UNIT_MA)
        return self._last_state

    def _write_goal_position(self, raw):
        with self._io_lock:
            self.packet.write4ByteTxRx(self.port, self.dxl_id, ADDR_GOAL_POSITION, int(raw))

    @staticmethod
    def _deg_to_pos(deg):
        return int(deg * 4096 / 360)

    # ================== angle <-> stroke ==================
    def angle_to_stroke(self, angle):
        return float(np.interp(angle, self.cal_angle, self.cal_stroke))

    def stroke_to_angle(self, stroke):
        return float(np.interp(stroke, self.cal_stroke[::-1], self.cal_angle[::-1]))

    # ================== motion ==================
    def _make_detector(self):
        p = self.get_parameter
        return ContactDetector(
            threshold_ma=p('contact_threshold_ma').value,
            n_samples=p('contact_samples').value,
            ignore_s=p('contact_ignore_s').value,
            stall_window_s=p('stall_window_s').value,
            stall_tol_deg=p('stall_tol_deg').value)

    def _on_contact(self, pos, ang, cur, goal_deg):
        self._contact = True
        self._contact_info = (ang, cur)
        if self.get_parameter('hold_mode').value == 'stop':
            self._write_goal_position(pos)
        else:
            # 닫는 방향으로 squeeze_offset_deg 더 → 위치 오차가 남아 Goal Current 한도까지 계속 쥠
            direction = 1.0 if goal_deg >= ang else -1.0
            hold_deg = ang + direction * self.get_parameter('squeeze_offset_deg').value
            self._write_goal_position(self._deg_to_pos(hold_deg))
        self.get_logger().info(
            f'접촉 감지: {ang:.1f} deg, stroke {self.angle_to_stroke(ang):.1f} mm, {cur:.0f} mA')

    def _move_to_angle(self, goal_deg, detect_contact=False, record_cb=None, phase=''):
        """goal_deg 까지 이동. 반환: 'arrived' | 'contact' | 'timeout' | 'stopped'"""
        goal = self._deg_to_pos(goal_deg)
        self._contact = False
        self._contact_info = None
        self._write_goal_position(goal)
        t_start = time.perf_counter()
        det = self._make_detector()
        det.reset(t_start)

        while rclpy.ok():
            if self._stop_flag.is_set():
                pos, _, _ = self._read_state()
                self._write_goal_position(pos)
                return 'stopped'

            pos, ang, cur = self._read_state()
            now = time.perf_counter()
            if record_cb:
                record_cb(phase, ang, cur)

            if abs(pos - goal) < self.arrive_tol:
                return 'arrived'

            if detect_contact and det.update(now, ang, cur):
                self._on_contact(pos, ang, cur, goal_deg)
                return 'contact'

            if now - t_start > self.move_timeout:
                self.get_logger().warn('시간 초과: 목표에 도달하지 못했습니다.')
                return 'timeout'
            time.sleep(self.sample_dt)
        return 'stopped'

    def _dwell(self, phase, record_cb):
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < self.dwell_time and rclpy.ok():
            _, ang, cur = self._read_state()
            record_cb(phase, ang, cur)
            time.sleep(self.sample_dt)

    def _run_exclusive(self, fn):
        if not self._motion_lock.acquire(blocking=False):
            return False, '다른 동작이 실행 중입니다.'
        try:
            self._stop_flag.clear()
            return fn()
        finally:
            self._motion_lock.release()

    # ================== service / topic callbacks ==================
    def _srv_open(self, req, res):
        def job():
            st = self._move_to_angle(self.open_angle)
            return st == 'arrived', f'open: {st}'
        res.success, res.message = self._run_exclusive(job)
        return res

    def _srv_close(self, req, res):
        def job():
            detect = self.get_parameter('contact_detect').value
            st = self._move_to_angle(self.close_angle, detect_contact=detect)
            if st == 'contact' and self._contact_info:
                ang, cur = self._contact_info
                msg = (f'close: contact | at {ang:.1f} deg, '
                       f'stroke {self.angle_to_stroke(ang):.1f} mm, {cur:.0f} mA')
            else:
                _, ang, cur = self._read_state()
                msg = (f'close: {st} | {ang:.1f} deg, '
                       f'stroke {self.angle_to_stroke(ang):.1f} mm, {cur:.0f} mA')
            return st in ('arrived', 'contact'), msg
        res.success, res.message = self._run_exclusive(job)
        return res

    def _srv_stop(self, req, res):
        self._stop_flag.set()
        res.success, res.message = True, '정지 요청'
        return res

    def _on_goal_stroke(self, msg):
        def job():
            ang = self.stroke_to_angle(float(msg.data))
            lo, hi = sorted((self.open_angle, self.close_angle))
            ang = min(max(ang, lo), hi)
            detect = self.get_parameter('contact_detect').value
            st = self._move_to_angle(ang, detect_contact=detect)
            return st in ('arrived', 'contact'), st
        ok, m = self._run_exclusive(job)
        self.get_logger().info(f'goal_stroke {msg.data:.1f} mm -> {m}')

    def _srv_run_test(self, req, res):
        res.success, res.message = self._run_exclusive(self._run_test_sequence)
        return res

    def _run_test_sequence(self):
        """open -> close -> open (무부하 실험용, 접촉 감지 없음), CSV 저장."""
        rows = []
        t0 = time.perf_counter()

        def rec(phase, ang, cur):
            rows.append((phase, time.perf_counter() - t0, ang, self.angle_to_stroke(ang), cur))

        status = []
        for goal, phase in ((self.open_angle, 'open1'),
                            (self.close_angle, 'close'),
                            (self.open_angle, 'open2')):
            st = self._move_to_angle(goal, record_cb=rec, phase=phase)
            status.append(f'{phase}:{st}')
            if st == 'stopped':
                break
            self._dwell(phase, rec)

        out_dir = Path(self.get_parameter('csv_dir').value)
        out_dir.mkdir(parents=True, exist_ok=True)
        fname = out_dir / datetime.now().strftime('gripper_time_stroke_%Y%m%d_%H%M%S.csv')
        with open(fname, 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.writer(f)
            w.writerow(['phase', 'time_s', 'motor_angle_deg', 'stroke_mm', 'current_mA'])
            for r in rows:
                w.writerow([r[0], f'{r[1]:.4f}', f'{r[2]:.3f}', f'{r[3]:.3f}', f'{r[4]:.1f}'])
        return True, f'{", ".join(status)} | CSV: {fname}'

    # ================== publishing ==================
    def _publish_state(self):
        _, ang, cur = self._read_state()
        self.pub_stroke.publish(Float32(data=self.angle_to_stroke(ang)))
        self.pub_current.publish(Float32(data=float(cur)))
        self.pub_contact.publish(Bool(data=self._contact))
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper_motor']
        js.position = [float(np.deg2rad(ang))]
        self.pub_js.publish(js)

    # ================== shutdown ==================
    def shutdown_motor(self):
        try:
            self._stop_flag.set()
            if self.get_parameter('torque_off_on_shutdown').value:
                pos, _, _ = self._read_state()
                self._write_goal_position(pos)
                with self._io_lock:
                    self.packet.write1ByteTxRx(self.port, self.dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().info('토크 OFF')
            else:
                self.get_logger().info('토크 유지 (쥐고 있던 힘 유지). 끄려면 전원을 내리거나 '
                                       'torque_off_on_shutdown:=true 로 실행')
            self.port.closePort()
            self.get_logger().info('포트 종료')
        except Exception as e:  # noqa
            self.get_logger().error(f'종료 처리 중 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_motor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
