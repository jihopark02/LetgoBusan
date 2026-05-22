import json
import os
import threading
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

_DB_PATH = os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', 'params', 'inventory_db_shelf.yaml'
)

# 재고 임계값 (이 수량 이하면 "보충 필요" 판단)
THRESHOLDS = {
    "banana": 2,
    "apple":  2,
    "grape":  2,
}

SYSTEM_PROMPT = """\
당신은 창고 드론 재고 조사 결과를 분석하는 AI 보고서 작성자입니다.

DB 기준 재고(예상값)와 실제 드론 스캔 결과를 비교하여 자연스러운 한국어 보고서를 작성하세요.

## 보고서 작성 규칙
- 임무가 무엇이었는지 한 문장으로 시작하세요.
- DB에 등록된 품목이 실제로 스캔되었는지 비교하세요. 스캔되지 않은 품목은 "미확인 (분실 가능성)"으로 언급하세요.
- 신선도가 "위험"이거나 재고 상태가 "긴급"인 품목은 반드시 강조하세요.
- 임계값 이하인 품목은 "보충 필요"라고 언급하세요.
- 전체 요약으로 마무리하세요.
- 3~5문장 분량으로 간결하게 작성하세요.

## 출력 형식 (JSON만 반환, 다른 텍스트 없이)
{"report": "완성된 한국어 보고서 문장"}
"""


def _load_db_by_zone(db_path: str) -> dict:
    """DB에서 구역별 기대 재고를 로드. 반환: {zone: [{location, item, qty}, ...]}"""
    candidates = [
        db_path,
        os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', '..', '..', 'params', 'inventory_db_shelf.yaml'
        )),
        '/home/jiho/capstone_ws/src/warehouse_offboard/params/inventory_db_shelf.yaml',
    ]
    for path in candidates:
        if os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                db = data.get('inventory_by_barcode', {})
                by_zone: dict = {}
                for _, info in db.items():
                    loc = info.get('expected_location', '')
                    zone = loc.rsplit('-L', 1)[0] if '-L' in loc else loc
                    by_zone.setdefault(zone, []).append({
                        'location': loc,
                        'item': info.get('item_name', ''),
                        'qty': info.get('quantity', 0),
                    })
                return by_zone
            except Exception:
                pass
    return {}


class ResultReportNode(Node):
    def __init__(self):
        super().__init__('result_report_node')

        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경변수가 설정되지 않았습니다.')
        self._client = OpenAI(api_key=api_key) if api_key else None

        self._db_by_zone = _load_db_by_zone(_DB_PATH)
        if self._db_by_zone:
            self.get_logger().info('재고 DB 로드 완료')
        else:
            self.get_logger().warn('재고 DB 로드 실패 — 비교 기능 비활성화')

        self._mission_description: str = ''
        self._mission_type: str = ''
        self._expected_targets: list = []
        self._finished_targets: set = set()
        self._scan_records: list = []
        self._report_lock = threading.Lock()
        self._reported = False

        self.create_subscription(String, '/llm/scan_report',    self._scan_report_cb, 10)
        self.create_subscription(String, '/mission_status_text', self._status_cb,     10)
        self.create_subscription(String, '/llm/mission_command', self._command_cb,    10)

        self._pub = self.create_publisher(String, '/llm/final_report', 10)

        self.get_logger().info('result_report_node 시작')

    def _scan_report_cb(self, msg: String):
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if record.get('filter_match'):
            self._scan_records.append(record)

    def _command_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            self._mission_type = cmd.get('mission_type', '')
            self._expected_targets = cmd.get('targets', [])
            self._mission_description = cmd.get('response', '임무 수행')
            self._finished_targets = set()
            self._scan_records = []
            self._reported = False
            self.get_logger().info(
                f'새 미션 수신: {self._mission_type} / 대상: {self._expected_targets}'
            )
        except json.JSONDecodeError:
            pass

    def _status_cb(self, msg: String):
        status = msg.data

        if status.startswith('MISSION_STARTED'):
            self._finished_targets = set()
            self._reported = False
            return

        if status.startswith('MISSION_FINISHED'):
            parts = status.split(':')
            wp = parts[1] if len(parts) > 1 else ''
            self._finished_targets.add(wp)
            self.get_logger().info(
                f'웨이포인트 완료: {wp} '
                f'({len(self._finished_targets)}/{len(self._expected_targets)})'
            )
            self._check_and_report()

    def _check_and_report(self):
        if self._mission_type != 'inventory_scan':
            return
        if not self._expected_targets:
            return
        if not set(self._expected_targets).issubset(self._finished_targets):
            return

        with self._report_lock:
            if self._reported:
                return
            self._reported = True

        self.get_logger().info('전체 웨이포인트 완료 — LLM 보고서 생성 중...')
        threading.Thread(target=self._call_llm, daemon=True).start()

    def _call_llm(self):
        if self._client is None:
            self.get_logger().error('OpenAI 클라이언트 없음')
            return

        # DB 기준 재고 (기대값)
        db_lines = []
        for zone in self._expected_targets:
            for entry in self._db_by_zone.get(zone, []):
                db_lines.append(
                    f"  {entry['location']}: {entry['item']} (기대수량={entry['qty']})"
                )
        db_section = '\n'.join(db_lines) if db_lines else '  (없음)'

        # 실제 스캔 결과
        scan_lines = []
        for r in self._scan_records:
            scan_lines.append(
                f"  {r.get('location', '?')}: {r.get('item', '?')} "
                f"(수량={r.get('qty')}, 신선도={r.get('freshness', '-')}, "
                f"재고상태={r.get('stock_status', '-')})"
            )
        scan_section = '\n'.join(scan_lines) if scan_lines else '  (스캔 결과 없음)'

        user_message = (
            f"임무: \"{self._mission_description}\"\n"
            f"조사 구역: {self._expected_targets}\n"
            f"\n[DB 기준 재고 (기대값)]\n{db_section}\n"
            f"\n[실제 스캔 결과]\n{scan_section}\n"
            f"\n재고 임계값: {json.dumps(THRESHOLDS, ensure_ascii=False)}"
        )

        try:
            resp = self._client.chat.completions.create(
                model='gpt-5-mini',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user',   'content': user_message},
                ],
                response_format={'type': 'json_object'},
            )

            raw = resp.choices[0].message.content.strip()
            result = json.loads(raw)
            report = result.get('report', '보고서 생성 실패')

            self.get_logger().info(f'최종 보고서: {report}')
            self._pub.publish(String(data=report))

        except Exception as e:
            self.get_logger().error(f'LLM 호출 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ResultReportNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
