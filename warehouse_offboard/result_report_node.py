import json
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

# 재고 임계값 (이 수량 이하면 "보충 필요" 판단)
THRESHOLDS = {
    "banana": 2,
    "apple":  2,
    "grape":  2,
}

SYSTEM_PROMPT = """\
당신은 창고 드론 재고 조사 결과를 분석하는 AI 보고서 작성자입니다.

아래 세 가지 정보를 바탕으로 자연스러운 한국어 보고서를 작성하세요.
1. 드론이 수행한 임무 내용
2. 실제 조사된 품목별 재고 수량
3. 품목별 최소 재고 임계값

## 보고서 작성 규칙
- 임무가 무엇이었는지 한 문장으로 시작하세요.
- 품목별 재고 수량을 자연스럽게 나열하세요.
- 임계값 이하인 품목은 반드시 "보충 필요"라고 언급하세요.
- 재고가 0개인 품목은 특히 강조하세요.
- 전체 요약으로 마무리하세요.
- 2~4문장 분량으로 간결하게 작성하세요.

## 출력 형식 (JSON만 반환, 다른 텍스트 없이)
{"report": "완성된 한국어 보고서 문장"}

## 예시
임무: "전체 창고 재고 조사"
결과: {"banana": 3, "apple": 1, "grape": 0}
임계값: {"banana": 2, "apple": 2, "grape": 2}

{"report": "전체 창고 재고 조사가 완료되었습니다. 바나나 3개, 사과 1개를 확인했으며, 포도는 재고가 없는 상태입니다. 사과(1개)와 포도(0개)는 최소 재고 기준 이하로 즉시 보충이 필요합니다."}
"""


class ResultReportNode(Node):
    def __init__(self):
        super().__init__('result_report_node')

        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경변수가 설정되지 않았습니다.')
        self._client = OpenAI(api_key=api_key) if api_key else None

        # 상태 저장
        self._latest_inventory: dict = {}
        self._mission_description: str = ''   # llm_node의 response 필드 재활용
        self._mission_type: str = ''
        self._expected_targets: list = []
        self._finished_targets: set = set()
        self._report_lock = threading.Lock()
        self._reported = False                # 한 미션당 한 번만 보고

        self.create_subscription(String, '/inventory/status',    self._inventory_cb, 10)
        self.create_subscription(String, '/mission_status_text', self._status_cb,    10)
        self.create_subscription(String, '/llm/mission_command', self._command_cb,   10)

        self._pub = self.create_publisher(String, '/llm/final_report', 10)

        self.get_logger().info('result_report_node 시작')

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _inventory_cb(self, msg: String):
        try:
            self._latest_inventory = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _command_cb(self, msg: String):
        """llm_node 가 발행한 미션 명령 수신 → 기대 웨이포인트 목록 저장."""
        try:
            cmd = json.loads(msg.data)
            self._mission_type = cmd.get('mission_type', '')
            self._expected_targets = cmd.get('targets', [])
            self._mission_description = cmd.get('response', '임무 수행')
            self._finished_targets = set()
            self._reported = False
            self.get_logger().info(
                f'새 미션 수신: {self._mission_type} / 대상: {self._expected_targets}'
            )
        except json.JSONDecodeError:
            pass

    def _status_cb(self, msg: String):
        status = msg.data

        # 새 미션 시작 시 초기화
        if status.startswith('MISSION_STARTED'):
            self._finished_targets = set()
            self._reported = False
            return

        # 웨이포인트 완료 감지
        if status.startswith('MISSION_FINISHED'):
            # "MISSION_FINISHED:A-01" → "A-01"
            parts = status.split(':')
            wp = parts[1] if len(parts) > 1 else ''
            self._finished_targets.add(wp)
            self.get_logger().info(
                f'웨이포인트 완료: {wp} '
                f'({len(self._finished_targets)}/{len(self._expected_targets)})'
            )
            self._check_and_report()

    # ── 보고서 생성 트리거 ────────────────────────────────────────────────

    def _check_and_report(self):
        """모든 기대 웨이포인트가 완료됐을 때 보고서 생성."""
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

    # ── LLM 호출 ──────────────────────────────────────────────────────────

    def _call_llm(self):
        if self._client is None:
            self.get_logger().error('OpenAI 클라이언트 없음')
            return

        user_message = (
            f"임무: \"{self._mission_description}\"\n"
            f"조사 결과: {json.dumps(self._latest_inventory, ensure_ascii=False)}\n"
            f"재고 임계값: {json.dumps(THRESHOLDS, ensure_ascii=False)}"
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
