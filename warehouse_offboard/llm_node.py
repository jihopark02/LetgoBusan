import json
import os
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from openai import OpenAI

# ── 창고 웨이포인트 정보 ──────────────────────────────────────────────────────
WAYPOINT_INFO = {
    "A-01": "1구역 — 우측 하단 선반",
    "A-02": "2구역 — 좌측 하단 선반",
    "A-03": "3구역 — 우측 상단 선반",
    "A-04": "4구역 — 좌측 상단 선반",
}

DB_PATH = os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', 'params', 'inventory_db_shelf.yaml'
)

_BASE_SYSTEM_PROMPT = """\
You are a mission planning AI for a warehouse drone inventory system.

## Warehouse Zone Information
A-01 : Zone 1, bottom-right shelf
A-02 : Zone 2, bottom-left shelf
A-03 : Zone 3, top-right shelf
A-04 : Zone 4, top-left shelf

{db_section}

## Mission Type
- inventory_scan : Visit the specified zones in order, scan barcodes, and aggregate inventory counts
- goto : Move to a specific zone only, no scanning
- status : Report current drone status only, no flight

## Response Rules
- Always return JSON only, no other text allowed
- The targets array must contain only valid values (A-01, A-02, A-03, A-04)
- item_filter: set this field only when a specific item name is explicitly mentioned, otherwise null
- If item_filter is set, targets must include only the zones where that item is stored
- Map user expressions to item_name in the DB using inference (e.g., "grapes" → item_filter: "포도")
- Even if the user uses varied expressions for zone numbers, directions, locations, or item names, infer the correct zone(s) from context and the DB
- The response value must be a natural Korean sentence to be shown to the user

## Output Format
{{"mission_type": "...", "targets": [...], "item_filter": null, "response": "..."}}

## 예시
User: "창고 전체 재고 조사해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01", "A-02", "A-03", "A-04"], "item_filter": null, "response": "전체 4개 구역을 순회하며 재고를 조사합니다."}}

User: "1구역 전체 재고조사 해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01"], "item_filter": null, "response": "1구역(A-01)으로 이동하여 재고를 조사합니다."}}

User: "우측 선반 다 확인해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01", "A-03"], "item_filter": null, "response": "우측 구역(A-01, A-03)을 순서대로 조사합니다."}}

User: "2번 구역 가봐"
{{"mission_type": "goto", "targets": ["A-02"], "item_filter": null, "response": "2구역(A-02)으로 이동합니다."}}

User: "지금 어디야?"
{{"mission_type": "status", "targets": [], "item_filter": null, "response": "현재 드론 상태를 확인합니다."}}

User: "포도 박스만 스캔해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01"], "item_filter": "포도", "response": "포도가 보관된 A-01 구역을 스캔합니다."}}

User: "딸기랑 수박 재고 확인해줘"
{{"mission_type": "inventory_scan", "targets": ["A-02"], "item_filter": "딸기,수박", "response": "딸기와 수박이 있는 A-02 구역을 스캔합니다."}}
"""


def _build_db_section(db_path: str) -> str:
    try:
        abs_path = os.path.normpath(os.path.join(os.path.dirname(__file__), db_path[3:] if db_path.startswith('../') else db_path))
        # Try direct path first
        candidates = [
            db_path,
            os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'params', 'inventory_db_shelf.yaml')),
            '/home/jiho/capstone_ws/src/warehouse_offboard/params/inventory_db_shelf.yaml',
        ]
        data = None
        for path in candidates:
            if os.path.exists(path):
                with open(path) as f:
                    data = yaml.safe_load(f)
                break
        if not data:
            return ''
        db = data.get('inventory_by_barcode', {})
        lines = ['## 재고 DB (SKU → 품목·위치)']
        for sku, info in db.items():
            loc = info.get('expected_location', '')
            item = info.get('item_name', '')
            qty = info.get('quantity', '')
            lines.append(f'- {sku} : {item}, 위치={loc}, 수량={qty}')
        return '\n'.join(lines)
    except Exception:
        return ''


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경변수가 설정되지 않았습니다.')
        self._client = OpenAI(api_key=api_key) if api_key else None

        db_section = _build_db_section(DB_PATH)
        self._system_prompt = _BASE_SYSTEM_PROMPT.format(db_section=db_section)
        if db_section:
            self.get_logger().info('재고 DB를 시스템 프롬프트에 로드했습니다.')
        else:
            self.get_logger().warn('재고 DB 로드 실패 — 품목 기반 추론 비활성화')

        # ── 드론 상태 ─────────────────────────────────────────────────────
        self._mission_status = 'IDLE'
        self._pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._history: list[dict] = []
        self._last_report: str = ''

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(String, '/mission_status_text', self._status_cb, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._pos_cb, px4_qos)
        self.create_subscription(String, '/llm/user_input', self._user_input_cb, 10)
        self.create_subscription(String, '/llm/final_report', self._report_cb, 10)

        self._cmd_pub = self.create_publisher(String, '/llm/mission_command', 10)
        self._resp_pub = self.create_publisher(String, '/llm/response_text', 10)

        self.get_logger().info('llm_node 시작 (/llm/user_input 구독 중)')

    def _status_cb(self, msg: String):
        self._mission_status = msg.data

    def _pos_cb(self, msg: VehicleLocalPosition):
        self._pos = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(-msg.z),
        }

    def _user_input_cb(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return
        self.get_logger().info(f'사용자 입력 수신: "{user_text}"')
        threading.Thread(target=self._call_llm, args=(user_text,), daemon=True).start()

    def _report_cb(self, msg: String):
        self._last_report = msg.data

    def _build_user_message(self, user_text: str) -> str:
        report_section = f"[최근 스캔 보고서] {self._last_report}\n" if self._last_report else ''
        return (
            f"[드론 상태] 미션={self._mission_status} | "
            f"위치 x={self._pos['x']:.1f}m, y={self._pos['y']:.1f}m, "
            f"고도={self._pos['z']:.1f}m\n"
            f"{report_section}"
            f"[사용자 명령] {user_text}"
        )

    def _call_llm(self, user_text: str):
        if self._client is None:
            self.get_logger().error('OpenAI 클라이언트 없음 — OPENAI_API_KEY 확인')
            return

        raw = ''
        try:
            user_msg = {'role': 'user', 'content': self._build_user_message(user_text)}
            messages = [{'role': 'system', 'content': self._system_prompt}] + self._history + [user_msg]

            resp = self._client.chat.completions.create(
                model='gpt-5-mini',
                messages=messages,
                response_format={'type': 'json_object'},
            )

            raw = resp.choices[0].message.content.strip()
            self.get_logger().debug(f'LLM 원시 응답: {raw}')

            result = json.loads(raw)
            mission_type = result.get('mission_type', '')
            targets = result.get('targets', [])
            item_filter = result.get('item_filter') or None
            response_text = result.get('response', '명령을 처리했습니다.')

            if mission_type not in ('inventory_scan', 'goto', 'status'):
                raise ValueError(f'알 수 없는 mission_type: {mission_type}')

            valid_targets = [t for t in targets if t in WAYPOINT_INFO]
            if targets and not valid_targets:
                raise ValueError(f'유효하지 않은 targets: {targets}')

            output = {
                'mission_type': mission_type,
                'targets': valid_targets,
                'item_filter': item_filter,
                'response': response_text,
            }
            output_str = json.dumps(output, ensure_ascii=False)

            self._history.append(user_msg)
            self._history.append({'role': 'assistant', 'content': raw})
            self._history = self._history[-10:]  # 최근 5턴 유지

            self.get_logger().info(f'LLM 출력 → /llm/mission_command: {output_str}')
            self._cmd_pub.publish(String(data=output_str))
            self._resp_pub.publish(String(data=response_text))

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 파싱 실패: {e} / 원문: {raw}')
        except ValueError as e:
            self.get_logger().error(f'검증 실패: {e}')
        except Exception as e:
            self.get_logger().error(f'LLM 호출 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
