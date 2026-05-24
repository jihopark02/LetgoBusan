#!/usr/bin/env python3

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleLandDetected
from std_msgs.msg import String


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GotoPoint(Node):
    def __init__(self):
        super().__init__('goto_point')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos
        )

        # PX4 subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            px4_qos
        )
        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.vehicle_land_detected_callback,
            px4_qos
        )

        # Mission UI pub/sub
        self.mission_target_subscriber = self.create_subscription(
            String,
            '/mission_target_name',
            self.mission_target_callback,
            10
        )

        _latching_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.mission_status_publisher = self.create_publisher(
            String,
            '/mission_status_text',
            _latching_qos
        )

        # inventory result subscriber
        self.inventory_result_subscriber = self.create_subscription(
            String,
            '/inventory_scan_result',
            self.inventory_result_callback,
            10
        )

        # aruco_land 상태 구독
        self.aruco_land_done = False
        self.create_subscription(
            String,
            '/aruco_land/status',
            self.aruco_land_status_callback,
            10
        )
        self.create_subscription(String, '/mission_cancel', self._cancel_cb, 10)

        # ---------- parameters ----------
        self.declare_parameter('reach_tolerance', 0.30)
        self.declare_parameter('hover_time_sec', 3.0)
        self.declare_parameter('preland_hover_time_sec', 2.0)
        self.declare_parameter('preland_xy_tolerance', 0.10)

        self.declare_parameter('spawn_world_x', -10.0)
        self.declare_parameter('spawn_world_y', 0.0)
        self.declare_parameter('yaw_align_deg', 90.0)
        self.declare_parameter('safe_transit_z', -2.5)

        self.declare_parameter('waypoint_names', ['A-01', 'A-02', 'A-03', 'A-04'])
        self.declare_parameter('waypoint_world_x', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('waypoint_world_y', [2.5, -1.5, -5.5, -7.5])
        self.declare_parameter('waypoint_z', [-2.0, -2.0, -2.0, -2.0])

        self.declare_parameter('scan_layer_z', [-0.65, -1.05, -1.45, -1.85])
        self.declare_parameter('scan_layer_hover_time_sec', 2.0)
        self.declare_parameter('scan_layer_timeout_sec', 8.0)
        self.declare_parameter('scan_require_inventory_result', True)

        # WAIT_ARUCO_LAND 타임아웃 (초). 이 시간 안에 ARUCO_LAND_DONE 안 오면 강제로 FINISHED 전환
        self.declare_parameter('aruco_land_timeout_sec', 30.0)

        # YAML 호환성 유지용
        self.declare_parameter('target_name', 'A-01')

        self.reach_tolerance = float(self.get_parameter('reach_tolerance').value)
        self.hover_time_sec = float(self.get_parameter('hover_time_sec').value)
        self.preland_hover_time_sec = float(self.get_parameter('preland_hover_time_sec').value)
        self.preland_xy_tolerance = float(self.get_parameter('preland_xy_tolerance').value)

        self.spawn_world_x = float(self.get_parameter('spawn_world_x').value)
        self.spawn_world_y = float(self.get_parameter('spawn_world_y').value)
        self.yaw_align_deg = float(self.get_parameter('yaw_align_deg').value)
        self.safe_transit_z = float(self.get_parameter('safe_transit_z').value)

        self.default_target_name = str(self.get_parameter('target_name').value)

        self.scan_layer_z = [float(v) for v in list(self.get_parameter('scan_layer_z').value)]
        if len(self.scan_layer_z) != 4:
            raise ValueError('scan_layer_z must have exactly 4 values for L1~L4')

        self.scan_layer_hover_time_sec = float(self.get_parameter('scan_layer_hover_time_sec').value)
        self.scan_layer_timeout_sec = float(self.get_parameter('scan_layer_timeout_sec').value)
        self.scan_require_inventory_result = bool(self.get_parameter('scan_require_inventory_result').value)

        self.aruco_land_timeout_sec = float(self.get_parameter('aruco_land_timeout_sec').value)
        self.aruco_land_timeout_limit = max(1, int(self.aruco_land_timeout_sec / 0.1))

        self.scan_layer_hover_limit = max(1, int(self.scan_layer_hover_time_sec / 0.1))
        self.scan_layer_timeout_limit = max(1, int(self.scan_layer_timeout_sec / 0.1))

        waypoint_names = list(self.get_parameter('waypoint_names').value)
        waypoint_world_x = list(self.get_parameter('waypoint_world_x').value)
        waypoint_world_y = list(self.get_parameter('waypoint_world_y').value)
        waypoint_z = list(self.get_parameter('waypoint_z').value)

        n = len(waypoint_names)
        if not (
            len(waypoint_world_x) == n and
            len(waypoint_world_y) == n and
            len(waypoint_z) == n
        ):
            raise ValueError(
                'waypoint_names, waypoint_world_x, waypoint_world_y, waypoint_z length mismatch'
            )

        self.waypoint_map: Dict[str, List[float]] = {}
        for i in range(n):
            self.waypoint_map[str(waypoint_names[i]).upper()] = [
                float(waypoint_world_x[i]),
                float(waypoint_world_y[i]),
                float(waypoint_z[i]),
            ]

        # ---------- state ----------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_heading = 0.0

        self.position_valid = False
        self.received_position_once = False
        self.landed = False

        self.home_initialized = False
        self.home_x: Optional[float] = None
        self.home_y: Optional[float] = None
        self.home_z: Optional[float] = None
        self.home_yaw: Optional[float] = None

        self.target_name: Optional[str] = None
        self.target_world: Optional[List[float]] = None
        self.target_local_x: Optional[float] = None
        self.target_local_y: Optional[float] = None
        self.target_local_z: Optional[float] = None

        self.pending_target_name: Optional[str] = None
        self.pending_layer_queue: List[int] = [0, 1, 2, 3]
        self.last_finished_target: Optional[str] = None

        self.phase = 'WAIT_HOME'
        self.prev_phase = ''

        self.offboard_setpoint_counter = 0
        self.hover_counter = 0
        self.hover_limit = max(1, int(self.hover_time_sec / 0.1))

        self.preland_hover_counter = 0
        self.preland_hover_limit = max(1, int(self.preland_hover_time_sec / 0.1))

        self.land_command_sent = False
        self.disarm_sent = False

        # 4-layer inventory scan state
        self.scan_index = 0          # index into scan_layer_queue
        self.scan_layer_queue: List[int] = [0, 1, 2, 3]  # 0-indexed layer numbers to visit
        self.scan_hover_counter = 0
        self.scan_wait_counter = 0
        self.scan_results: Dict[str, Dict] = {}
        self.last_inventory_msg: Optional[Dict] = None

        # WAIT_ARUCO_LAND 타임아웃 카운터
        self.aruco_land_wait_counter = 0

        self._launch_home: dict | None = None
        self._from_interrupt: bool = False

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('goto_point node started [4-LAYER INVENTORY SCAN MODE]')
        self.get_logger().info('Waiting for valid local position before first mission...')
        self.get_logger().info(f'scan_layer_z={self.scan_layer_z}')
        self.publish_mission_status('WAITING_FOR_COMMAND')

    # ------------------------------------------------------------------
    # mission status / callbacks
    # ------------------------------------------------------------------
    def publish_mission_status(self, text: str):
        msg = String()
        msg.data = text
        self.mission_status_publisher.publish(msg)

    def aruco_land_status_callback(self, msg: String):
        if msg.data == 'ARUCO_LAND_DONE':
            self.get_logger().info('aruco_land 정밀 착륙 완료 신호 수신')
            self.aruco_land_done = True

    def _cancel_cb(self, msg: String):
        if self.phase in ['WAIT_HOME', 'FINISHED', 'INTERRUPT_HOVER']:
            return
        self._from_interrupt = not self.landed
        self.publish_mission_status(f'MISSION_FINISHED:{self.target_name}')
        self.phase = 'INTERRUPT_HOVER'
        self.get_logger().info(
            f'미션 인터럽트: {self.target_name} → INTERRUPT_HOVER '
            f'(공중={self._from_interrupt})'
        )

    def inventory_result_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'Invalid inventory result JSON: {msg.data}')
            return

        self.last_inventory_msg = data

        expected_location = str(data.get('expected_location_from_db') or '').upper()
        barcode = str(data.get('barcode_detected') or '').upper()

        if not expected_location:
            return

        # 현재 목표 선반과 관련 없는 결과는 무시
        if self.target_name is not None and not expected_location.startswith(self.target_name):
            return

        # 현재 스캔 중인 층과 맞는 결과만 저장
        expected_layer = self.current_scan_location()
        if expected_layer is not None and expected_location == expected_layer:
            self.scan_results[expected_layer] = data
            self.get_logger().info(
                f'Inventory result accepted for {expected_layer}: barcode={barcode}'
            )
            self.publish_mission_status(f'INVENTORY_ACCEPTED:{expected_layer}')
        else:
            self.get_logger().info(
                f'Inventory result ignored: expected_location={expected_location}, '
                f'current_scan={expected_layer}'
            )

    def mission_target_callback(self, msg: String):
        raw = msg.data.strip()

        # JSON 형식 {"zone": "A-01", "layers": [1,2,3,4]} 또는 plain 문자열 "A-01" 지원
        try:
            parsed = json.loads(raw)
            target_name = parsed.get('zone', '').upper()
            layers_1indexed = parsed.get('layers', [1, 2, 3, 4])
            pending_layer_queue = [int(l) - 1 for l in layers_1indexed if 1 <= int(l) <= 4]
        except (json.JSONDecodeError, ValueError):
            target_name = raw.upper()
            pending_layer_queue = [0, 1, 2, 3]

        if not pending_layer_queue:
            pending_layer_queue = [0, 1, 2, 3]

        if target_name not in self.waypoint_map:
            self.get_logger().warn(f'Unknown mission target received: {target_name}')
            self.publish_mission_status('MISSION_REJECTED:UNKNOWN_TARGET')
            return

        if self.phase not in ['WAIT_HOME', 'FINISHED', 'INTERRUPT_HOVER']:
            self.get_logger().warn(
                f'Received {target_name}, but mission is already running (phase={self.phase})'
            )
            self.publish_mission_status('MISSION_REJECTED:BUSY')
            return

        self.pending_target_name = target_name
        self.pending_layer_queue = pending_layer_queue
        self.get_logger().info(
            f'Pending mission target set: {target_name}, layers={[l+1 for l in pending_layer_queue]}'
        )
        self.publish_mission_status(f'명령 수신: {target_name}')

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        if not self.received_position_once:
            self.get_logger().info(
                f'Local position received | xy_valid={msg.xy_valid}, '
                f'z_valid={msg.z_valid}, x={msg.x:.2f}, y={msg.y:.2f}, '
                f'z={msg.z:.2f}, heading={msg.heading:.2f}'
            )
            self.received_position_once = True

        self.current_x = float(msg.x)
        self.current_y = float(msg.y)
        self.current_z = float(msg.z)
        self.current_heading = float(msg.heading)
        self.position_valid = bool(msg.xy_valid and msg.z_valid)

    def vehicle_land_detected_callback(self, msg: VehicleLandDetected):
        self.landed = bool(msg.landed)

    # ------------------------------------------------------------------
    # mission start
    # ------------------------------------------------------------------
    def start_new_mission(self, selected_target_name: str):
        if not self.position_valid:
            self.get_logger().warn('Cannot start new mission: local position is invalid')
            self.publish_mission_status('MISSION_REJECTED:POSITION_INVALID')
            return

        # 최초 미션 시 이착륙 지점 저장 (이후 변경 안 함)
        if self._launch_home is None:
            self._launch_home = {
                'x': self.current_x, 'y': self.current_y,
                'z': self.current_z, 'yaw': self.current_heading,
            }

        # 항상 최초 이착륙 지점을 기준으로 사용 (드리프트 누적 방지)
        self.home_x = self._launch_home['x']
        self.home_y = self._launch_home['y']
        self.home_z = self._launch_home['z']
        self.home_yaw = self._launch_home['yaw']
        self.home_initialized = True

        if self._from_interrupt:
            # 공중 인터럽트: 안전 고도로 상승 후 타겟으로 이동
            start_phase = 'ASCEND_SAFE'
            start_counter = 11
            self._from_interrupt = False
        else:
            start_phase = 'TAKEOFF'
            start_counter = 0

        self.target_name = selected_target_name
        self.target_world = self.waypoint_map[self.target_name]

        self.target_local_x, self.target_local_y = self.world_to_local_xy(
            self.target_world[0], self.target_world[1]
        )
        self.target_local_z = self.scan_layer_z[0]

        # ── 상태 완전 초기화 ──
        self.phase = start_phase
        self.prev_phase = ''
        self.offboard_setpoint_counter = start_counter
        self.hover_counter = 0
        self.preland_hover_counter = 0
        self.land_command_sent = False
        self.disarm_sent = False
        self.aruco_land_done = False          # 이전 미션의 ARUCO_LAND_DONE 잔류 방지
        self.aruco_land_wait_counter = 0     # WAIT_ARUCO_LAND 타임아웃 카운터 초기화

        self.scan_index = 0
        self.scan_layer_queue = list(self.pending_layer_queue)
        self.scan_hover_counter = 0
        self.scan_wait_counter = 0
        self.scan_results = {}
        self.last_inventory_msg = None

        self.get_logger().info(
            f'New 4-layer inventory mission selected: {self.target_name} -> world {self.target_world}'
        )
        self.get_logger().info(
            f'New mission local shelf target XY: ({self.target_local_x:.2f}, '
            f'{self.target_local_y:.2f}), scan_z={self.scan_layer_z}'
        )
        self.get_logger().info(
            f'New mission home: ({self.home_x:.2f}, {self.home_y:.2f}, '
            f'{self.home_z:.2f}), yaw={self.home_yaw:.2f}'
        )

        self.publish_mission_status(f'MISSION_STARTED:{self.target_name}')

    # ------------------------------------------------------------------
    # coordinate transform
    # ------------------------------------------------------------------
    def world_to_local_xy(self, goal_world_x: float, goal_world_y: float) -> Tuple[float, float]:
        if self.home_x is None or self.home_y is None:
            raise ValueError('home local position is not initialized')

        dx_world = goal_world_x - self.spawn_world_x
        dy_world = goal_world_y - self.spawn_world_y

        target_local_x = self.home_x + dy_world
        target_local_y = self.home_y + dx_world
        return target_local_x, target_local_y

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def aligned_yaw(self) -> float:
        if self.home_yaw is None:
            return 0.0
        return normalize_angle(self.home_yaw + math.radians(self.yaw_align_deg))

    def current_scan_location(self) -> Optional[str]:
        if self.target_name is None:
            return None
        if self.scan_index < 0 or self.scan_index >= len(self.scan_layer_queue):
            return None
        layer_num = self.scan_layer_queue[self.scan_index]
        return f'{self.target_name}-L{layer_num + 1}'

    def current_scan_z(self) -> float:
        if self.scan_index < len(self.scan_layer_queue):
            layer_num = self.scan_layer_queue[self.scan_index]
        else:
            layer_num = self.scan_layer_queue[-1] if self.scan_layer_queue else 0
        return self.scan_layer_z[max(0, min(3, layer_num))]

    def reached_x(self, target_x: float, tolerance: Optional[float] = None) -> bool:
        tol = self.reach_tolerance if tolerance is None else tolerance
        return abs(self.current_x - target_x) < tol

    def reached_y(self, target_y: float, tolerance: Optional[float] = None) -> bool:
        tol = self.reach_tolerance if tolerance is None else tolerance
        return abs(self.current_y - target_y) < tol

    def reached_z(self, target_z: float, tolerance: Optional[float] = None) -> bool:
        tol = self.reach_tolerance if tolerance is None else tolerance
        return abs(self.current_z - target_z) < tol

    def reached_yaw(self, target_yaw: float, tolerance: float = 0.08) -> bool:
        return abs(normalize_angle(self.current_heading - target_yaw)) < tolerance

    def reached_home_xy_precise(self) -> bool:
        if self.home_x is None or self.home_y is None:
            return False
        return (
            abs(self.current_x - self.home_x) < self.preland_xy_tolerance and
            abs(self.current_y - self.home_y) < self.preland_xy_tolerance
        )

    def reached_home_yaw(self) -> bool:
        if self.home_yaw is None:
            return False
        return self.reached_yaw(self.home_yaw, tolerance=0.08)

    def compute_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def print_scan_summary(self):
        self.get_logger().info('\n===== 선반 재고 조사 요약 =====')
        if self.target_name is None:
            return

        for layer_num in self.scan_layer_queue:
            loc = f'{self.target_name}-L{layer_num + 1}'
            data = self.scan_results.get(loc)
            if data is None:
                self.get_logger().info(f'{loc}: 결과 없음')
            else:
                self.get_logger().info(
                    f"{loc}: barcode={data.get('barcode_detected')}, "
                    f"품명={data.get('item_name')}, "
                    f"수량={data.get('quantity')}, "
                    f"가격={data.get('price')}, "
                    f"입고일={data.get('inbound_date')}"
                )
        self.get_logger().info('===============================')

    # ------------------------------------------------------------------
    # publishers
    # ------------------------------------------------------------------
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    # ------------------------------------------------------------------
    # phase target
    # ------------------------------------------------------------------
    def get_phase_target(self) -> Optional[List[float]]:
        if self.phase == 'INTERRUPT_HOVER':
            return [self.current_x, self.current_y, self.current_z, self.current_heading]

        if (
            self.home_x is None or self.home_y is None or self.home_z is None or
            self.home_yaw is None or
            self.target_local_x is None or self.target_local_y is None
        ):
            return None

        if self.phase == 'TAKEOFF':
            return [self.home_x, self.home_y, self.scan_layer_z[0], self.home_yaw]

        if self.phase == 'YAW_ALIGN':
            return [self.home_x, self.home_y, self.scan_layer_z[0], self.aligned_yaw()]

        if self.phase == 'ASCEND_SAFE':
            return [self.current_x, self.current_y, self.safe_transit_z, self.aligned_yaw()]

        if self.phase == 'MOVE_SAFE':
            return [self.target_local_x, self.target_local_y, self.safe_transit_z, self.aligned_yaw()]

        if self.phase == 'MOVE_GLOBAL_Y':
            return [self.target_local_x, self.home_y, self.scan_layer_z[0], self.aligned_yaw()]

        if self.phase == 'MOVE_GLOBAL_X':
            return [self.target_local_x, self.target_local_y, self.scan_layer_z[0], self.aligned_yaw()]

        if self.phase == 'SCAN_LAYER':
            return [self.target_local_x, self.target_local_y, self.current_scan_z(), self.aligned_yaw()]

        if self.phase == 'RETURN_GLOBAL_X':
            return [self.target_local_x, self.home_y, self.scan_layer_z[0], self.aligned_yaw()]

        if self.phase == 'RETURN_GLOBAL_Y':
            return [self.home_x, self.home_y, self.scan_layer_z[0], self.aligned_yaw()]

        if self.phase == 'PRELAND_YAW_HOME':
            return [self.home_x, self.home_y, self.scan_layer_z[0], self.home_yaw]

        if self.phase == 'PRELAND_SETTLE':
            return [self.home_x, self.home_y, self.scan_layer_z[0], self.home_yaw]

        return None

    # ------------------------------------------------------------------
    # main timer
    # ------------------------------------------------------------------
    def timer_callback(self):
        # ── WAIT_HOME / FINISHED: 다음 미션 대기 ──
        if self.phase == 'WAIT_HOME':
            if not self.position_valid:
                self.get_logger().warn('Waiting for valid local position...')
                return
            if self.pending_target_name is not None:
                selected = self.pending_target_name
                self.pending_target_name = None
                self.start_new_mission(selected)
            return

        if self.phase == 'FINISHED':
            # 다음 미션 명령이 들어오면 즉시 새 미션 시작
            if self.pending_target_name is not None:
                selected = self.pending_target_name
                self.pending_target_name = None
                self.get_logger().info(f'FINISHED 상태에서 새 미션 수신: {selected} → 미션 재시작')
                self.start_new_mission(selected)
            return

        if self.phase == 'INTERRUPT_HOVER':
            # offboard 유지하며 현재 위치 고정, 새 타겟 대기
            self.publish_offboard_control_mode()
            target_pose = self.get_phase_target()
            if target_pose is not None:
                self.publish_trajectory_setpoint(
                    target_pose[0], target_pose[1], target_pose[2], target_pose[3]
                )
            if self.pending_target_name is not None:
                selected = self.pending_target_name
                self.pending_target_name = None
                self.start_new_mission(selected)
            return

        # ── offboard control mode & setpoint 발행 ──
        if self.phase not in ['WAIT_DISARM', 'WAIT_ARUCO_LAND']:
            self.publish_offboard_control_mode()

            target_pose = self.get_phase_target()
            if target_pose is not None:
                self.publish_trajectory_setpoint(
                    target_pose[0], target_pose[1], target_pose[2], target_pose[3]
                )

            if self.offboard_setpoint_counter == 10:
                self.get_logger().info('Switching to OFFBOARD mode')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0
                )
                self.get_logger().info('Arming vehicle')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0
                )

            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
                return

        if self.phase != self.prev_phase:
            self.get_logger().info(f'Phase changed -> {self.phase}')
            self.prev_phase = self.phase

        if self.phase not in ['WAIT_DISARM', 'WAIT_ARUCO_LAND']:
            target_pose = self.get_phase_target()
            if target_pose is not None:
                distance = self.compute_distance(
                    self.current_x, self.current_y, self.current_z,
                    target_pose[0], target_pose[1], target_pose[2]
                )
                self.get_logger().info(
                    f'[{self.phase}] '
                    f'current=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) | '
                    f'target=({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f}) | '
                    f'dist={distance:.2f}'
                )

        if self.phase == 'ASCEND_SAFE':
            if self.reached_z(self.safe_transit_z):
                self.phase = 'MOVE_SAFE'

        elif self.phase == 'MOVE_SAFE':
            if self.reached_x(self.target_local_x) and self.reached_y(self.target_local_y):
                self.phase = 'SCAN_LAYER'
                self.scan_index = 0
                self.scan_hover_counter = 0
                self.scan_wait_counter = 0
                self.publish_mission_status(f'SCAN_START:{self.target_name}')

        elif self.phase == 'TAKEOFF':
            if self.reached_z(self.scan_layer_z[0]):
                self.phase = 'YAW_ALIGN'

        elif self.phase == 'YAW_ALIGN':
            if self.reached_yaw(self.aligned_yaw(), tolerance=0.12):
                self.phase = 'MOVE_GLOBAL_Y'

        elif self.phase == 'MOVE_GLOBAL_Y':
            if self.reached_x(self.target_local_x):
                self.phase = 'MOVE_GLOBAL_X'

        elif self.phase == 'MOVE_GLOBAL_X':
            if self.reached_y(self.target_local_y):
                self.phase = 'SCAN_LAYER'
                self.scan_index = 0
                self.scan_hover_counter = 0
                self.scan_wait_counter = 0
                self.publish_mission_status(f'SCAN_START:{self.target_name}')

        elif self.phase == 'SCAN_LAYER':
            current_loc = self.current_scan_location()
            current_z = self.current_scan_z()

            if self.reached_z(current_z):
                self.scan_hover_counter += 1
                self.scan_wait_counter += 1

                if self.scan_hover_counter == 1:
                    self.get_logger().info(f'Scanning {current_loc} at z={current_z:.2f}')
                    self.publish_mission_status(f'SCAN_LAYER:{current_loc}')

                got_result = current_loc in self.scan_results

                if got_result:
                    self.get_logger().info(f'Scan result received for {current_loc}')
                elif self.scan_wait_counter >= self.scan_layer_timeout_limit:
                    self.get_logger().warn(f'Scan timeout for {current_loc}')
                    self.publish_mission_status(f'SCAN_TIMEOUT:{current_loc}')

                hover_done = self.scan_hover_counter >= self.scan_layer_hover_limit
                timeout_done = self.scan_wait_counter >= self.scan_layer_timeout_limit

                if hover_done and (got_result or timeout_done or not self.scan_require_inventory_result):
                    self.scan_index += 1
                    self.scan_hover_counter = 0
                    self.scan_wait_counter = 0

                    if self.scan_index >= len(self.scan_layer_queue):
                        self.print_scan_summary()
                        self.publish_mission_status(f'SCAN_DONE:{self.target_name}')
                        self.phase = 'RETURN_GLOBAL_X'
                    else:
                        next_loc = self.current_scan_location()
                        self.get_logger().info(f'Moving to next layer: {next_loc}')
                        self.publish_mission_status(f'SCAN_NEXT:{next_loc}')
            else:
                self.scan_hover_counter = 0

        elif self.phase == 'RETURN_GLOBAL_X':
            if self.reached_y(self.home_y):
                self.phase = 'RETURN_GLOBAL_Y'

        elif self.phase == 'RETURN_GLOBAL_Y':
            if self.reached_x(self.home_x):
                self.phase = 'PRELAND_YAW_HOME'

        elif self.phase == 'PRELAND_YAW_HOME':
            if self.reached_home_yaw():
                self.phase = 'PRELAND_SETTLE'

        elif self.phase == 'PRELAND_SETTLE':
            if self.reached_home_xy_precise():
                self.preland_hover_counter += 1
                self.get_logger().info(
                    f'[PRELAND_SETTLE] home_err_xy='
                    f'({self.current_x - self.home_x:.3f}, {self.current_y - self.home_y:.3f}), '
                    f'yaw_err={normalize_angle(self.current_heading - self.home_yaw):.3f}'
                )
                if self.preland_hover_counter >= self.preland_hover_limit:
                    self.preland_hover_counter = 0
                    self.get_logger().info('PRELAND_SETTLE 완료 → aruco_land 에 제어 넘김')
                    self.publish_mission_status('PRELAND_SETTLE')
                    self.phase = 'WAIT_ARUCO_LAND'
                    self.aruco_land_wait_counter = 0  # 타임아웃 카운터 시작
            else:
                self.preland_hover_counter = 0

        elif self.phase == 'WAIT_ARUCO_LAND':
            self.aruco_land_wait_counter += 1

            if self.aruco_land_done:
                # 정상 케이스: aruco_land가 완료 신호를 보냄
                self.get_logger().info('aruco_land 완료 확인 → FINISHED')
                self._finish_mission()

            elif self.aruco_land_wait_counter >= self.aruco_land_timeout_limit:
                # 타임아웃 케이스: ARUCO_LAND_DONE이 안 와도 FINISHED로 강제 전환
                self.get_logger().warn(
                    f'WAIT_ARUCO_LAND 타임아웃 ({self.aruco_land_timeout_sec}s) → 강제 FINISHED 전환'
                )
                self.publish_mission_status('ARUCO_LAND_TIMEOUT')
                self._finish_mission()

            else:
                # 타임아웃 경고 로그 (10초마다)
                if self.aruco_land_wait_counter % 100 == 0:
                    elapsed = self.aruco_land_wait_counter * 0.1
                    remaining = self.aruco_land_timeout_sec - elapsed
                    self.get_logger().info(
                        f'WAIT_ARUCO_LAND 대기 중... {elapsed:.0f}s 경과 '
                        f'(타임아웃까지 {remaining:.0f}s)'
                    )

        elif self.phase == 'LAND_CMD':
            if not self.land_command_sent:
                self.get_logger().info('Sending NAV_LAND command (fallback)')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_command_sent = True
                self.phase = 'WAIT_DISARM'

        elif self.phase == 'WAIT_DISARM':
            if self.landed and not self.disarm_sent:
                self.get_logger().info('Touchdown detected, disarming')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0
                )
                self.disarm_sent = True
                self._finish_mission()

    def _finish_mission(self):
        """미션 완료 처리 - FINISHED 상태로 전환하고 다음 미션 대기"""
        self.last_finished_target = self.target_name
        self.phase = 'FINISHED'
        self.publish_mission_status(f'MISSION_FINISHED:{self.last_finished_target}')
        self.get_logger().info(
            f'미션 완료: {self.last_finished_target} → FINISHED 상태. 다음 미션 명령을 기다립니다.'
        )
        # UI에 준비 완료 상태 알림
        self.publish_mission_status('WAITING_FOR_COMMAND')


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoint()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('goto_point node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
