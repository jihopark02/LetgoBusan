import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionSequencer(Node):
    def __init__(self):
        super().__init__('mission_sequencer')

        self._queue = []
        self._active = None
        self._pending_queue = []

        self.create_subscription(String, '/llm/mission_command', self._cmd_cb, 10)
        self.create_subscription(String, '/mission_status_text', self._status_cb, 10)
        self._target_pub = self.create_publisher(String, '/mission_target_name', 10)
        self._cancel_pub = self.create_publisher(String, '/mission_cancel', 10)

        self.get_logger().info('mission_sequencer 시작 — /llm/mission_command 대기 중')

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'JSON 파싱 실패: {msg.data}')
            return

        mission_type = data.get('mission_type', '')
        targets = data.get('targets', [])

        if mission_type == 'status' or not targets:
            self.get_logger().info(f'비행 없는 미션 타입: {mission_type}')
            return

        if self._active is not None:
            # 비행 중 → 인터럽트: _active 유지 (MISSION_FINISHED 감지용)
            self._pending_queue = list(targets)
            self._cancel_pub.publish(String(data='cancel'))
            self.get_logger().info(f'미션 인터럽트 요청: 현재={self._active}, 대기={self._pending_queue}')
        else:
            self._queue = list(targets)
            self.get_logger().info(f'미션 큐 등록: {self._queue}')
            self._send_next()

    def _status_cb(self, msg: String):
        status = msg.data
        if not status.startswith('MISSION_FINISHED:'):
            return
        finished = status.split(':', 1)[1]
        if finished == self._active:
            self.get_logger().info(f'완료 확인: {finished} — 다음 타겟으로')
            self._active = None
            if self._pending_queue:
                self._queue = self._pending_queue
                self._pending_queue = []
                self.get_logger().info(f'인터럽트 큐 처리: {self._queue}')
            self._send_next()

    def _send_next(self):
        if not self._queue:
            self.get_logger().info('모든 미션 완료')
            return
        target = self._queue.pop(0)
        self._active = target
        self._target_pub.publish(String(data=target))
        self.get_logger().info(f'발행 → /mission_target_name: {target}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('mission_sequencer 종료')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
