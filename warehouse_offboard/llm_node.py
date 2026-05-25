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
- Map user expressions to item_name in the DB using inference (e.g., "grapes" → item_filter: "grape")
- Even if the user uses varied expressions for zone numbers, directions, locations, or item names, infer the correct zone(s) from context and the DB
- The response value must be a natural Korean sentence to be shown to the user
- If the user asks about inventory quantities, prices, or locations (e.g., "how many X?", "what is the price of X?", "where is X?"), answer directly from the DB with mission_type: "status" — do NOT trigger inventory_scan
- Only use inventory_scan when the user explicitly requests a physical scan (e.g., "scan", "check", "verify", "스캔해줘", "재고 조사")

## scan_layers Rules
- scan_layers specifies which shelf layers (1~4) to scan per zone
- If item_filter is set: look up the DB to find which exact layer each item is on, then set scan_layers to only those layers
- If item_filter is null (full zone scan): set scan_layers to null
- scan_layers format: {{"A-01": [1, 3], "A-03": [2]}} — layer numbers are 1-indexed (L1=1, L2=2, L3=3, L4=4)
- Multiple items in the same zone: combine their layers, e.g. grape(L1) + strawberry(L3) in A-01 → {{"A-01": [1, 3]}}

## Output Format
{{"mission_type": "...", "targets": [...], "scan_layers": null, "item_filter": null, "response": "..."}}

## 예시
User: "창고 전체 재고 조사해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01", "A-02", "A-03", "A-04"], "scan_layers": null, "item_filter": null, "response": "전체 4개 구역을 순회하며 재고를 조사합니다."}}

User: "1구역 전체 재고조사 해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01"], "scan_layers": null, "item_filter": null, "response": "1구역(A-01)으로 이동하여 재고를 조사합니다."}}

User: "우측 선반 다 확인해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01", "A-03"], "scan_layers": null, "item_filter": null, "response": "우측 구역(A-01, A-03)을 순서대로 조사합니다."}}

User: "2번 구역 가봐"
{{"mission_type": "goto", "targets": ["A-02"], "scan_layers": null, "item_filter": null, "response": "2구역(A-02)으로 이동합니다."}}

User: "[드론 상태] 미션=IDLE | 위치 x=5.2m, y=3.1m, 고도=2.0m\n[사용자 명령] 지금 어디야?"
{{"mission_type": "status", "targets": [], "scan_layers": null, "item_filter": null, "response": "현재 드론은 x=5.2m, y=3.1m, 고도 2.0m 위치에서 대기 중입니다."}}

User: "포도 스캔해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01"], "scan_layers": {{"A-01": [1]}}, "item_filter": "grape", "response": "포도가 있는 A-01 구역 1층만 스캔합니다."}}

User: "포도랑 티셔츠 재고 확인해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01", "A-03"], "scan_layers": {{"A-01": [1], "A-03": [1]}}, "item_filter": "grape,t-shirt", "response": "포도(A-01 L1)와 티셔츠(A-03 L1) 위치만 스캔합니다."}}

User: "포도랑 딸기 확인해줘"
{{"mission_type": "inventory_scan", "targets": ["A-01"], "scan_layers": {{"A-01": [1, 3]}}, "item_filter": "grape,strawberry", "response": "A-01 구역의 포도(L1)와 딸기(L3)만 스캔합니다."}}

User: "go home"
{{"mission_type": "status", "targets": [], "scan_layers": null, "item_filter": null, "response": "드론은 미션 완료 후 자동으로 홈 위치로 복귀합니다. 별도의 홈 이동 명령은 없습니다."}}

User: "포도 몇 개야?"
{{"mission_type": "status", "targets": [], "scan_layers": null, "item_filter": null, "response": "DB 기준 포도(grape)는 A-01 구역 1층에 21개 있습니다."}}

User: "what is the most expensive thing in this warehouse?"
{{"mission_type": "status", "targets": [], "scan_layers": null, "item_filter": null, "response": "DB 기준 가장 비싼 물품은 수박(watermelon)으로 12,000원입니다(A-01 L4)."}}"""


def _build_db_section(db_path: str) -> str:
    try:
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
            price = info.get('price', '')
            lines.append(f'- {sku} : {item}, 위치={loc}, 수량={qty}, 가격={price}원')
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
        self._history_lock = threading.Lock()
        self._last_report: str = ''
        self._last_scan_item: str = ''

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
        self.create_subscription(String, '/llm/scan_report', self._scan_report_cb, 10)
        self.create_subscription(String, '/inventory_scan_result', self._inventory_result_cb, 10)

        self._cmd_pub = self.create_publisher(String, '/llm/mission_command', 10)
        self._resp_pub = self.create_publisher(String, '/llm/response_text', 10)

        self.get_logger().info('llm_node 시작 (/llm/user_input 구독 중)')

    _REJECTED_MSG = {
        'BUSY':             '현재 미션이 진행 중입니다. 완료 후 다시 시도해주세요.',
        'UNKNOWN_TARGET':   '알 수 없는 목표 구역입니다.',
        'POSITION_INVALID': '드론 위치 정보가 유효하지 않습니다.',
    }

    def _status_cb(self, msg: String):
        self._mission_status = msg.data
        if msg.data.startswith('MISSION_REJECTED:'):
            reason = msg.data.split(':', 1)[1]
            text = self._REJECTED_MSG.get(reason, '명령이 거부되었습니다.')
            self._resp_pub.publish(String(data=text))

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

    def _scan_report_cb(self, msg: String):
        self._last_scan_item = msg.data  # LLM 컨텍스트용

    def _inventory_result_cb(self, msg: String):
        """QR 감지 즉시 실시간 표시 — GPT 분석 딜레이 없음"""
        try:
            data = json.loads(msg.data)
            item     = data.get('item_name', '')
            qty      = data.get('quantity', '')
            location = data.get('expected_location_from_db', '')
            if item and qty:
                self._resp_pub.publish(String(data=f'[스캔] {location} | {item} | 수량 {qty}개'))
        except (json.JSONDecodeError, AttributeError):
            pass

    def _build_user_message(self, user_text: str) -> str:
        report_section = f"[최근 스캔 보고서] {self._last_report}\n" if self._last_report else ''
        scan_section = f"[실시간 스캔 항목] {self._last_scan_item}\n" if self._last_scan_item else ''
        return (
            f"[드론 상태] 미션={self._mission_status} | "
            f"위치 x={self._pos['x']:.1f}m, y={self._pos['y']:.1f}m, "
            f"고도={self._pos['z']:.1f}m\n"
            f"{scan_section}"
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
            with self._history_lock:
                messages = [{'role': 'system', 'content': self._system_prompt}] + list(self._history) + [user_msg]

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
            scan_layers = result.get('scan_layers') or None
            response_text = result.get('response', '명령을 처리했습니다.')

            if mission_type not in ('inventory_scan', 'goto', 'status'):
                raise ValueError(f'알 수 없는 mission_type: {mission_type}')

            valid_targets = [t for t in targets if t in WAYPOINT_INFO]
            if targets and not valid_targets:
                raise ValueError(f'유효하지 않은 targets: {targets}')

            if scan_layers:
                scan_layers = {
                    zone: sorted(set(int(l) for l in layers if 1 <= int(l) <= 4))
                    for zone, layers in scan_layers.items()
                    if zone in WAYPOINT_INFO and layers
                }
                if not scan_layers:
                    scan_layers = None

            output = {
                'mission_type': mission_type,
                'targets': valid_targets,
                'scan_layers': scan_layers,
                'item_filter': item_filter,
                'response': response_text,
            }
            output_str = json.dumps(output, ensure_ascii=False)

            with self._history_lock:
                self._history.append(user_msg)
                self._history.append({'role': 'assistant', 'content': raw})
                self._history = self._history[-10:]

            self.get_logger().info(f'LLM 출력 → /llm/mission_command: {output_str}')
            self._cmd_pub.publish(String(data=output_str))
            self._resp_pub.publish(String(data=response_text))

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 파싱 실패: {e} / 원문: {raw}')
            self._resp_pub.publish(String(data='응답 처리 중 오류가 발생했습니다. 다시 시도해주세요.'))
        except ValueError as e:
            self.get_logger().error(f'검증 실패: {e}')
            self._resp_pub.publish(String(data='명령을 처리할 수 없습니다. 다시 말씀해주세요.'))
        except Exception as e:
            self.get_logger().error(f'LLM 호출 오류: {e}')
            self._resp_pub.publish(String(data='서비스 연결에 실패했습니다. 잠시 후 다시 시도해주세요.'))


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
