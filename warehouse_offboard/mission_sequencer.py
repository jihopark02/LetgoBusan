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
        scan_layers = data.get('scan_layers') or {}

        if mission_type == 'status' or not targets:
            self.get_logger().info(f'비행 없는 미션 타입: {mission_type}')
            return

        # 각 target을 {"zone": "A-01", "layers": [1,2,3,4]} 형태로 인코딩
        def encode(zone):
            layers = scan_layers.get(zone) or [1, 2, 3, 4]
            return json.dumps({'zone': zone, 'layers': layers}, ensure_ascii=False)

        if self._active is not None:
            self._pending_queue = [encode(t) for t in targets]
            self._cancel_pub.publish(String(data='cancel'))
            self.get_logger().info(f'미션 인터럽트 요청: 현재={self._active}, 대기={self._pending_queue}')
        else:
            self._queue = [encode(t) for t in targets]
            self.get_logger().info(f'미션 큐 등록: {self._queue}')
            self._send_next()

    def _status_cb(self, msg: String):
        status = msg.data
        if not status.startswith('MISSION_FINISHED:'):
            return
        finished = status.split(':', 1)[1]

        try:
            active_zone = json.loads(self._active).get('zone') if self._active else None
        except (json.JSONDecodeError, AttributeError):
            active_zone = self._active

        # 정상 완료(마지막 구역 일치) 또는 인터럽트(pending 있음) 둘 다 처리
        if finished == active_zone or self._pending_queue:
            self.get_logger().info(f'완료 확인: {finished} — 다음 배치로')
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

        # 전체 큐를 배치로 전송 — goto_point가 선반 간 연속 비행 처리
        targets = [json.loads(t) for t in self._queue]
        payload = json.dumps({'targets': targets}, ensure_ascii=False)

        # 마지막 구역을 MISSION_FINISHED 감지용으로 추적
        self._active = self._queue[-1]
        self._queue = []

        self._target_pub.publish(String(data=payload))
        self.get_logger().info(f'발행 → /mission_target_name (배치 {len(targets)}개): {payload}')


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
