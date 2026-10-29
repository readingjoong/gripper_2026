#!/usr/bin/env python3
"""
m1013_gripper_control / gripper_node.py   (ROS2 Humble, rclpy + dynamixel_sdk)

test_time_stroke.py 를 ROS2 노드로 변환한 버전.

Services (std_srvs/Trigger)
  /gripper/open       그리퍼 완전 열림
  /gripper/close      close_angle 까지 닫음 (contact_detect=True 면 전류 급증 시 정지)
  /gripper/stop       현재 위치에서 정지
  /gripper/run_test   open -> close -> open 시퀀스 실행 + CSV 저장 (무부하 실험용)

Subscribers
  /gripper/goal_stroke  (std_msgs/Float32)  목표 stroke [mm]

Publishers (10 Hz, 파라미터로 변경 가능)
  /gripper/stroke_mm    (std_msgs/Float32)
  /gripper/current_ma   (std_msgs/Float32)
  /gripper/joint_states (sensor_msgs/JointState)  position = 모터 각도 [rad]
"""
import csv
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
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


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # ---------- parameters ----------
        # WSL2 에서는 COM11 이 아니라 /dev/ttyUSB0 (usbipd attach 후) 로 보임
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4000000)
        self.declare_parameter('dxl_id', 3)

        # 최근 실험 설정(Goal Current 40, Vel 50, Acc 10)을 기본값으로 둠
        self.declare_parameter('goal_current', 40)       # raw
        self.declare_parameter('profile_acc', 10)
        self.declare_parameter('profile_vel', 50)

        self.declare_parameter('open_angle', 0.0)        # deg
        self.declare_parameter('close_angle', 600.0)     # deg
        self.declare_parameter('arrive_tol', 20)         # raw pulse
        self.declare_parameter('move_timeout', 30.0)     # s
        self.declare_parameter('dwell_time', 1.0)        # s (run_test)
        self.declare_parameter('sample_dt', 0.01)        # s

        # 접촉 감지 (close 서비스 / goal_stroke 에 적용)
        self.declare_parameter('contact_detect', False)
        self.declare_parameter('contact_threshold_ma', 80.0)
        self.declare_parameter('contact_samples', 3)     # 연속 N회 초과 시 접촉

        # angle -> stroke 보정표 (각도 오름차순)
        self.declare_parameter('cal_angle',
                               [8.0, 50.0, 104.0, 160.0, 232.0, 292.0,
                                373.0, 436.0, 488.0, 531.0, 600.0])
        self.declare_parameter('cal_stroke',
                               [90.0, 84.5, 77.0, 68.0, 57.5, 48.0,
                                35.0, 24.0, 16.0, 9.5, 0.0])

        self.declare_parameter('publish_rate', 10.0)     # Hz
        self.declare_parameter('csv_dir', str(Path.home() / 'gripper_logs'))
        self.declare_parameter('torque_off_on_shutdown', True)

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

        # ---------- state / locks ----------
        self._io_lock = threading.Lock()       # dynamixel 포트 접근 보호
        self._motion_lock = threading.Lock()   # 동시에 하나의 동작만 허용
        self._stop_flag = threading.Event()
        self._last_state = (0, 0.0, 0.0)       # (raw position, angle deg, current mA)

        # ---------- dynamixel ----------
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f'포트를 열 수 없습니다: {self.port_name}')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'Baudrate 설정 실패: {self.baudrate}')
        self.get_logger().info(f'포트 연결 성공: {self.port_name} @ {self.baudrate}')
        self._init_motor(p('goal_current').value,
                         p('profile_acc').value,
                         p('profile_vel').value)

        # ---------- ROS interfaces ----------
        cb = ReentrantCallbackGroup()
        self.pub_stroke = self.create_publisher(Float32, 'gripper/stroke_mm', 10)
        self.pub_current = self.create_publisher(Float32, 'gripper/current_ma', 10)
        self.pub_js = self.create_publisher(JointState, 'gripper/joint_states', 10)

        self.create_service(Trigger, 'gripper/open', self._srv_open, callback_group=cb)
        self.create_service(Trigger, 'gripper/close', self._srv_close, callback_group=cb)
        self.create_service(Trigger, 'gripper/stop', self._srv_stop, callback_group=cb)
        self.create_service(Trigger, 'gripper/run_test', self._srv_run_test, callback_group=cb)
        self.create_subscription(Float32, 'gripper/goal_stroke',
                                 self._on_goal_stroke, 10, callback_group=cb)

        self.create_timer(1.0 / p('publish_rate').value, self._publish_state,
                          callback_group=cb)
        self.get_logger().info('gripper_node 준비 완료')

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
        self.get_logger().warn(
            'Wizard 의 Current Limit(addr 38)이 goal_current 보다 낮으면 '
            '이동 중 전류가 제한되어 멈출 수 있습니다.')

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
        self._last_state = (pos, pos * 360.0 / 4096.0, cur * CURRENT_UNIT_MA)
        return self._last_state

    def _write_goal_position(self, raw):
        with self._io_lock:
            self.packet.write4ByteTxRx(self.port, self.dxl_id, ADDR_GOAL_POSITION, int(raw))

    @staticmethod
    def _deg_to_pos(deg):
        return int(deg * 4096 / 360)

    # ================== angle <-> stroke ==================
    def angle_to_stroke(self, angle):
        # np.interp 는 범위 밖에서 양 끝값을 유지 (원본 코드와 동일 동작)
        return float(np.interp(angle, self.cal_angle, self.cal_stroke))

    def stroke_to_angle(self, stroke):
        # stroke 는 각도에 대해 감소 -> 뒤집어서 보간
        return float(np.interp(stroke, self.cal_stroke[::-1], self.cal_angle[::-1]))

    # ================== motion ==================
    def _move_to_angle(self, goal_deg, detect_contact=False, record_cb=None, phase=''):
        """goal_deg 까지 이동. 반환: 'arrived' | 'contact' | 'timeout' | 'stopped'"""
        goal = self._deg_to_pos(goal_deg)
        self._write_goal_position(goal)
        p = self.get_parameter
        thr = p('contact_threshold_ma').value
        n_need = p('contact_samples').value
        over = 0
        t_start = time.perf_counter()

        while rclpy.ok():
            if self._stop_flag.is_set():
                pos, _, _ = self._read_state()
                self._write_goal_position(pos)
                return 'stopped'

            pos, ang, cur = self._read_state()
            if record_cb:
                record_cb(phase, ang, cur)

            if abs(pos - goal) < self.arrive_tol:
                return 'arrived'

            if detect_contact:
                over = over + 1 if abs(cur) > thr else 0
                if over >= n_need:
                    self._write_goal_position(pos)   # 현재 위치에서 홀드
                    return 'contact'

            if time.perf_counter() - t_start > self.move_timeout:
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
        """동작 중복 실행 방지 래퍼. 반환: (success, message)"""
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
            _, ang, cur = self._read_state()
            return st in ('arrived', 'contact'), (
                f'close: {st} | stroke {self.angle_to_stroke(ang):.1f} mm, {cur:.0f} mA')
        res.success, res.message = self._run_exclusive(job)
        return res

    def _srv_stop(self, req, res):
        self._stop_flag.set()
        res.success, res.message = True, '정지 요청'
        return res

    def _on_goal_stroke(self, msg):
        def job():
            ang = self.stroke_to_angle(float(msg.data))
            ang = min(max(ang, self.open_angle), self.close_angle)   # 안전 clamp
            detect = self.get_parameter('contact_detect').value
            st = self._move_to_angle(ang, detect_contact=detect)
            return st in ('arrived', 'contact'), st
        ok, m = self._run_exclusive(job)
        self.get_logger().info(f'goal_stroke {msg.data:.1f} mm -> {m}')

    def _srv_run_test(self, req, res):
        res.success, res.message = self._run_exclusive(self._run_test_sequence)
        return res

    def _run_test_sequence(self):
        """open -> close -> open, time/angle/stroke/current 기록 후 CSV 저장."""
        rows = []
        t0 = time.perf_counter()

        def rec(phase, ang, cur):
            rows.append((phase, time.perf_counter() - t0, ang,
                         self.angle_to_stroke(ang), cur))

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
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper_motor']
        js.position = [float(np.deg2rad(ang))]
        self.pub_js.publish(js)

    # ================== shutdown ==================
    def shutdown_motor(self):
        try:
            self._stop_flag.set()
            pos, _, _ = self._read_state()
            self._write_goal_position(pos)
            if self.get_parameter('torque_off_on_shutdown').value:
                with self._io_lock:
                    self.packet.write1ByteTxRx(self.port, self.dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.port.closePort()
            self.get_logger().info('모터 정지, 포트 종료')
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
