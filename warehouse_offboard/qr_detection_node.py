import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class QRDetectionNode(Node):
    def __init__(self):
        super().__init__('qr_detection_node')

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self._last_published = None
        self._cooldown_frames = 0
        self.COOLDOWN = 15

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)

        # 단순 품목명 (inv_counter_node 호환용)
        self.pub_result = self.create_publisher(String, '/qr/result', 10)
        # 전체 JSON 정보 (llm_scan_analyzer 용)
        self.pub_scan = self.create_publisher(String, '/qr/scan_result', 10)

        self.get_logger().info('qr_detection_node 시작 — /camera/image_raw 구독 중')

    def _image_cb(self, msg: Image):
        if self._cooldown_frames > 0:
            self._cooldown_frames -= 1
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 오류: {e}')
            return

        data, _, _ = self.detector.detectAndDecode(frame)
        if not data:
            return

        raw = data.strip()
        if raw == self._last_published:
            self._cooldown_frames = self.COOLDOWN
            return

        self._last_published = raw
        self._cooldown_frames = self.COOLDOWN

        # JSON 형식인지 확인
        try:
            parsed = json.loads(raw)
            item = parsed.get('item', raw).lower()
            self.get_logger().info(f'QR 인식 (JSON): {parsed}')
            self.pub_result.publish(String(data=item))        # 품목명만
            self.pub_scan.publish(String(data=raw))           # 전체 JSON
        except json.JSONDecodeError:
            # 단순 문자열 (하위 호환)
            item = raw.lower()
            self.get_logger().info(f'QR 인식 (텍스트): {item}')
            self.pub_result.publish(String(data=item))


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
