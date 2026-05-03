import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ITEMS = ["banana", "apple", "grape"]


class InvCounterNode(Node):
    def __init__(self):
        super().__init__('inv_counter_node')

        self._counts = {item: 0 for item in ITEMS}

        self.create_subscription(String, '/qr/result', self._qr_cb, 10)
        self.create_subscription(String, '/mission_status_text', self._status_cb, 10)
        self._pub = self.create_publisher(String, '/inventory/status', 10)

        self.get_logger().info('inv_counter_node 시작')

    def _qr_cb(self, msg: String):
        item = msg.data.strip().lower()
        if item not in self._counts:
            self.get_logger().warn(f'알 수 없는 품목: {item}')
            return
        self._counts[item] += 1
        self.get_logger().info(f'QR 집계: {item} → 현재 {self._counts}')
        self._publish()

    def _status_cb(self, msg: String):
        if msg.data.startswith('MISSION_STARTED'):
            self._counts = {item: 0 for item in ITEMS}
            self.get_logger().info('새 미션 시작 — 카운터 초기화')

    def _publish(self):
        self._pub.publish(String(data=json.dumps(self._counts, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = InvCounterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
