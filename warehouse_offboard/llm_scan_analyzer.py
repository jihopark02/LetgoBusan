import json
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

TODAY = datetime.now().strftime("%Y.%m.%d")

SYSTEM_PROMPT = f"""\
당신은 창고 드론 재고 스캔 분석 AI입니다.
오늘 날짜는 {TODAY}입니다.

드론이 스캔한 품목 정보를 받아 분석하고 아래 형식의 JSON을 반환하세요.

수량 기준이나 유통기한은 별도로 주어지지 않습니다.
품목의 일반적인 특성(유통기한, 회전율, 보관 민감도 등)을 스스로 판단하여 평가하세요.

## 출력 필드 설명
- days_elapsed  : 오늘 날짜 기준 입고일로부터 경과 일수 (정수)
- total_value   : 단가 × 수량 (정수, 원 단위)
- freshness     : 신선도 상태 — "양호" / "주의" / "위험" 중 하나
- stock_status  : 수량 상태 — "적정" / "부족" / "긴급" 중 하나
- recommendation: 한 줄 권고사항 (15자 이내)
- analysis      : 종합 분석 문장 (2~3문장)

## 출력 형식 (JSON만, 다른 텍스트 없이)
{{
  "days_elapsed": 0,
  "total_value": 0,
  "freshness": "양호",
  "stock_status": "적정",
  "recommendation": "추가 입고 불필요",
  "analysis": "분석 내용"
}}
"""


class LLMScanAnalyzer(Node):
    def __init__(self):
        super().__init__('llm_scan_analyzer')

        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경변수가 설정되지 않았습니다.')
        self._client = OpenAI(api_key=api_key) if api_key else None

        # 현재 미션에서 필터링할 품목 (None = 전체 스캔)
        self._item_filter: list[str] | None = None

        self.create_subscription(String, '/llm/mission_command',   self._mission_cmd_cb,  10)
        self.create_subscription(String, '/qr/scan_result',        self._scan_cb,         10)
        self.create_subscription(String, '/inventory_scan_result', self._vision_scan_cb,  10)
        self._pub_analysis = self.create_publisher(String, '/llm/scan_analysis', 10)
        self._pub_report   = self.create_publisher(String, '/llm/scan_report',   10)

        self.get_logger().info('llm_scan_analyzer 시작')

    def _mission_cmd_cb(self, msg: String):
        """llm_node의 mission_command에서 item_filter를 추출해 보관."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        raw_filter = data.get('item_filter')
        if raw_filter:
            # "포도,수박" 형태 또는 단일 문자열 모두 지원
            self._item_filter = [s.strip() for s in str(raw_filter).split(',') if s.strip()]
            self.get_logger().info(f'item_filter 설정: {self._item_filter}')
        else:
            self._item_filter = None

    def _matches_filter(self, item_name: str) -> bool:
        if self._item_filter is None:
            return True
        return any(f in item_name or item_name in f for f in self._item_filter)

    def _vision_scan_cb(self, msg: String):
        raw = msg.data.strip()
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f'JSON 파싱 실패: {raw}')
            return

        location = data.get('shelf_text_detected') or data.get('expected_location_from_db', '')
        if location and '-L' in location:
            location = location.rsplit('-L', 1)[0]

        normalized = {
            'location': location,
            'item':     data.get('item_name', ''),
            'qty':      data.get('quantity', 0),
            'price':    data.get('price', 0),
            'date':     data.get('inbound_date', ''),
        }
        self.get_logger().info(f'inventory_vision_shelf 데이터 수신: {normalized}')
        threading.Thread(target=self._call_llm, args=(normalized,), daemon=True).start()

    def _scan_cb(self, msg: String):
        raw = msg.data.strip()
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f'JSON 파싱 실패: {raw}')
            return

        self.get_logger().info(f'스캔 데이터 수신: {data}')
        threading.Thread(target=self._call_llm, args=(data,), daemon=True).start()

    def _call_llm(self, data: dict):
        if self._client is None:
            return

        item_name = data.get('item', '')
        filter_match = self._matches_filter(item_name)

        # 필터 불일치 품목은 GPT 분석 없이 스킵 레코드만 발행
        if not filter_match:
            self.get_logger().info(f'item_filter 불일치 → 스킵: {item_name}')
            skip_record = {
                'location':       data.get('location'),
                'item':           item_name,
                'qty':            data.get('qty'),
                'price':          data.get('price'),
                'date':           data.get('date'),
                'filter_match':   False,
                'days_elapsed':   0,
                'total_value':    0,
                'freshness':      '-',
                'stock_status':   '-',
                'recommendation': '필터 제외 품목',
                'analysis':       '',
            }
            self._pub_report.publish(String(data=json.dumps(skip_record, ensure_ascii=False)))
            return

        user_message = (
            f"위치: {data.get('location')}\n"
            f"품목: {item_name}\n"
            f"수량: {data.get('qty')}개\n"
            f"단가: {data.get('price')}원\n"
            f"입고일: {data.get('date')}"
        )

        try:
            resp = self._client.chat.completions.create(
                model='gpt-5-mini',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user',   'content': user_message},
                ],
                temperature=0.3,
                response_format={'type': 'json_object'},
            )

            raw = resp.choices[0].message.content.strip()
            result = json.loads(raw)
            analysis = result.get('analysis', '')
            self.get_logger().info(f'GPT 분석 결과: {analysis}')

            record = {
                'location':       data.get('location'),
                'item':           item_name,
                'qty':            data.get('qty'),
                'price':          data.get('price'),
                'date':           data.get('date'),
                'filter_match':   True,
                'days_elapsed':   result.get('days_elapsed', 0),
                'total_value':    result.get('total_value', 0),
                'freshness':      result.get('freshness', ''),
                'stock_status':   result.get('stock_status', ''),
                'recommendation': result.get('recommendation', ''),
                'analysis':       analysis,
            }
            record_str = json.dumps(record, ensure_ascii=False)
            self._pub_analysis.publish(String(data=analysis))
            self._pub_report.publish(String(data=record_str))

        except Exception as e:
            self.get_logger().error(f'LLM 호출 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMScanAnalyzer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
