#!/usr/bin/env python3
# aruco_land_v03 - 단일 마커, MISSION_STARTED 후에만 활성화
import math
import threading
from typing import Optional

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from cv_bridge import CvBridge
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                           VehicleLocalPosition, VehicleLandDetected)
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


class ArucoLandNode(Node):
    IDLE      = 'IDLE'
    SEARCHING = 'SEARCHING'
    ALIGNING  = 'ALIGNING'
    LANDING   = 'LANDING'
    DONE      = 'DONE'

    def __init__(self):
        super().__init__('aruco_land')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.declare_parameter('aruco_marker_id',     0)
        self.declare_parameter('aruco_marker_size_m', 0.3)
        self.declare_parameter('align_xy_tol_m',      0.15)
        self.declare_parameter('align_hold_sec',      1.0)
        self.declare_parameter('descent_speed_mps',   0.15)

        self.marker_id      = self.get_parameter('aruco_marker_id').value
        self.align_tol      = float(self.get_parameter('align_xy_tol_m').value)
        self.align_hold_sec = float(self.get_parameter('align_hold_sec').value)
        self.descent_speed  = float(self.get_parameter('descent_speed_mps').value)

        self.aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.state      = self.IDLE
        self.prev_state = ''
        self.mission_started = False  # MISSION_STARTED 후에만 활성화

        self.cur_x = self.cur_y = self.cur_z = self.cur_heading = 0.0
        self.pos_valid = False
        self.landed    = False

        self.camera_matrix = None
        self.dist_coeffs   = None

        self.sp_x = self.sp_y = self.sp_z = None
        self.sp_yaw = 0.0

        self.align_hold_counter = 0.0
        self.land_cmd_sent      = False
        self.disarm_sent        = False
        self._marker_aligned    = False

        self._lock  = threading.Lock()
        self.bridge = CvBridge()

        self.ocm_pub    = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.sp_pub     = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   px4_qos)
        self.cmd_pub    = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',        px4_qos)
        self.status_pub = self.create_publisher(String, '/aruco_land/status', 10)
        self.debug_pub  = self.create_publisher(Image,  '/aruco_land/debug_image', 10)

        _latching_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(String, '/mission_status_text', self.mission_status_cb, _latching_qos)
        self.create_subscription(Image,  '/camera/down_image_raw', self.image_cb, 10)
        self.create_subscription(CameraInfo, '/camera/down_camera_info', self.camera_info_cb, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, px4_qos)
        self.create_subscription(VehicleLandDetected,  '/fmu/out/vehicle_land_detected', self.land_detected_cb, px4_qos)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'aruco_land v03 ready | marker_id={self.marker_id} align_tol={self.align_tol}m')

    def mission_status_cb(self, msg: String):
        if msg.data.startswith('MISSION_STARTED:'):
            self.mission_started = True
            self.get_logger().info(f'미션 시작 감지: {msg.data}')
            return
        if msg.data == 'PRELAND_SETTLE' and self.state == self.IDLE:
            if not self.mission_started:
                self.get_logger().warn('PRELAND_SETTLE 무시 (MISSION_STARTED 전)')
                return
            self.mission_started = False
            self.get_logger().info('PRELAND_SETTLE 수신 → 정밀 착륙 시작!')
            with self._lock:
                self.sp_x   = self.cur_x
                self.sp_y   = self.cur_y
                self.sp_z   = self.cur_z
                self.sp_yaw = self.cur_heading
                self.align_hold_counter = 0.0
                self.land_cmd_sent = False
                self.disarm_sent   = False
                self.state = self.SEARCHING

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.cur_x=float(msg.x); self.cur_y=float(msg.y)
        self.cur_z=float(msg.z); self.cur_heading=float(msg.heading)
        self.pos_valid=bool(msg.xy_valid and msg.z_valid)

    def land_detected_cb(self, msg: VehicleLandDetected):
        self.landed = bool(msg.landed)
        if self.landed and self.state == self.LANDING:
            self.get_logger().info('착륙 감지 → DONE')
            self.state = self.DONE

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None: return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(f'CameraInfo 수신 | fx={self.camera_matrix[0,0]:.1f}')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'imgmsg_to_cv2: {e}'); return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        debug_frame = frame.copy()
        h, w = debug_frame.shape[:2]
        cv2.line(debug_frame, (w//2-30,h//2),(w//2+30,h//2),(0,255,0),2)
        cv2.line(debug_frame, (w//2,h//2-30),(w//2,h//2+30),(0,255,0),2)
        cv2.putText(debug_frame, f'state={self.state}', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(debug_frame, f'alt={abs(self.cur_z):.2f}m', (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        if ids is None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8'))
            if self.state == self.ALIGNING:
                with self._lock:
                    self._marker_aligned = False
                    self.align_hold_counter = 0.0
                    self.state = self.SEARCHING
            return

        target_idx = None
        for i, mid in enumerate(ids.flatten()):
            if mid == self.marker_id: target_idx = i; break
        if target_idx is None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')); return

        c  = corners[target_idx][0]
        cx = float(np.mean(c[:,0])); cy = float(np.mean(c[:,1]))
        err_px_x = cx - w/2.0; err_px_y = cy - h/2.0

        if self.camera_matrix is not None:
            fx=self.camera_matrix[0,0]; fy=self.camera_matrix[1,1]
            alt=max(abs(self.cur_z),0.1)
            err_m_x=(err_px_x/fx)*alt; err_m_y=(err_px_y/fy)*alt
        else:
            err_m_x=err_px_x*0.004; err_m_y=err_px_y*0.004

        err_total = math.sqrt(err_m_x**2+err_m_y**2)
        color = (0,255,0) if err_total < self.align_tol else (0,165,255)

        x1,y1=int(c[:,0].min()),int(c[:,1].min())
        x2,y2=int(c[:,0].max()),int(c[:,1].max())
        cv2.rectangle(debug_frame,(x1,y1),(x2,y2),color,3)
        corner_len=15
        for cx_,cy_,dx,dy in [(x1,y1,1,1),(x2,y1,-1,1),(x1,y2,1,-1),(x2,y2,-1,-1)]:
            cv2.line(debug_frame,(cx_,cy_),(cx_+dx*corner_len,cy_),color,3)
            cv2.line(debug_frame,(cx_,cy_),(cx_,cy_+dy*corner_len),color,3)
        label=f'ArUco ID:{self.marker_id}  err:{err_total:.3f}m'
        (lw,lh),_=cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,0.55,2)
        cv2.rectangle(debug_frame,(x1,y1-lh-10),(x1+lw+6,y1),color,-1)
        cv2.putText(debug_frame,label,(x1+3,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,0,0),2)
        cv2.circle(debug_frame,(int(cx),int(cy)),6,(0,0,255),-1)
        cv2.arrowedLine(debug_frame,(w//2,h//2),(int(cx),int(cy)),(255,0,0),2,tipLength=0.2)

        with self._lock:
            hold_now = self.align_hold_counter
        if err_total < self.align_tol and hold_now > 0:
            bar_w=int((hold_now/self.align_hold_sec)*(w-20))
            cv2.rectangle(debug_frame,(10,h-20),(w-10,h-10),(50,50,50),-1)
            cv2.rectangle(debug_frame,(10,h-20),(10+bar_w,h-10),(0,255,0),-1)
            cv2.putText(debug_frame,f'ALIGNED {hold_now:.1f}/{self.align_hold_sec}s',
                        (10,h-25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8'))

        if self.state not in (self.SEARCHING, self.ALIGNING): return

        with self._lock:
            if self.sp_x is None: return
            self.state = self.ALIGNING
            self._marker_aligned = err_total < self.align_tol
            self.get_logger().info(
                f'[ARUCO ✓] err={err_total:.3f}m aligned={self._marker_aligned}')

    def control_loop(self):
        if self.state == self.IDLE: return
        if self.state != self.prev_state:
            self.get_logger().info(f'[aruco_land] State → {self.state}')
            self.prev_state = self.state
            self._pub_status(self.state)

        if self.state in (self.SEARCHING, self.ALIGNING):
            self._pub_ocm()
            with self._lock:
                if self.sp_x is None: return
                if self._marker_aligned:
                    self.align_hold_counter += 0.1  # 10Hz timer → 0.1s per tick
                else:
                    self.align_hold_counter = 0.0
                if self.state == self.ALIGNING and self.align_hold_counter >= self.align_hold_sec:
                    self.get_logger().info('마커 확인 완료! → LANDING')
                    self.state = self.LANDING; return
                self.sp_z += self.descent_speed * 0.1
                self.sp_z = min(self.sp_z, -0.3)
                self.get_logger().info(f'[{self.state}] sp_z={self.sp_z:.2f} cur_z={self.cur_z:.2f}')
                self._pub_sp(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)
            return

        if self.state == self.LANDING:
            if not self.land_cmd_sent:
                self.get_logger().info('NAV_LAND 발행!')
                self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_cmd_sent = True
            return

        if self.state == self.DONE:
            if not self.disarm_sent:
                self.get_logger().info('착륙 완료 → Disarm')
                self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                self.disarm_sent = True
                self._pub_status('ARUCO_LAND_DONE')
                self.state = self.IDLE

    def _pub_ocm(self):
        msg=OffboardControlMode(); msg.timestamp=self.get_clock().now().nanoseconds//1000; msg.position=True
        self.ocm_pub.publish(msg)

    def _pub_sp(self,x,y,z,yaw):
        msg=TrajectorySetpoint(); msg.timestamp=self.get_clock().now().nanoseconds//1000
        msg.position=[float(x),float(y),float(z)]; msg.yaw=float(yaw)
        self.sp_pub.publish(msg)

    def _send_cmd(self,command,param1=0.0,param2=0.0):
        msg=VehicleCommand(); msg.timestamp=self.get_clock().now().nanoseconds//1000
        msg.param1=float(param1); msg.param2=float(param2); msg.command=int(command)
        msg.target_system=1; msg.target_component=1; msg.source_system=1; msg.source_component=1; msg.from_external=True
        self.cmd_pub.publish(msg)

    def _pub_status(self,text):
        msg=String(); msg.data=text; self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=ArucoLandNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('aruco_land stopped by user')
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
