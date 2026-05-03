import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SAVE_DIR = os.path.expanduser('~/capstone_ws')

COL_WIDTHS = {
    'location':       6,
    'item':           8,
    'qty':            4,
    'price':          9,
    'total_value':    10,
    'days_elapsed':   8,
    'freshness':      6,
    'stock_status':   6,
    'recommendation': 18,
}

HEADERS = {
    'location':       '위치',
    'item':           '품목',
    'qty':            '수량',
    'price':          '단가',
    'total_value':    '재고가치',
    'days_elapsed':   '경과',
    'freshness':      '신선도',
    'stock_status':   '재고상태',
    'recommendation': '권고사항',
}


def _cell(value, key):
    text = str(value)
    try:
        if key in ('price', 'total_value'):
            text = f"{int(value):,}원"
        elif key == 'days_elapsed':
            text = f"{value}일"
    except (ValueError, TypeError):
        pass
    width = COL_WIDTHS[key]
    # 한글 포함 시 실제 출력 폭 보정
    display_len = sum(2 if ord(c) > 127 else 1 for c in text)
    pad = width - display_len
    return text + ' ' * max(pad, 0)


def _print_table(records):
    keys = list(COL_WIDTHS.keys())
    sep = '┼'.join('─' * (COL_WIDTHS[k] + 2) for k in keys)

    header_cells = ' │ '.join(_cell(HEADERS[k], k) for k in keys)
    print('\n' + '┌' + '┬'.join('─' * (COL_WIDTHS[k] + 2) for k in keys) + '┐')
    print('│ ' + header_cells + ' │')
    print('├' + sep + '┤')
    for r in records:
        row_cells = ' │ '.join(_cell(r.get(k, ''), k) for k in keys)
        print('│ ' + row_cells + ' │')
    print('└' + '┴'.join('─' * (COL_WIDTHS[k] + 2) for k in keys) + '┘')


def _save_csv(records):
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(SAVE_DIR, f'inventory_report_{ts}.csv')
    keys = list(COL_WIDTHS.keys()) + ['analysis']
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(records)
    return path


class InventoryReporter(Node):
    def __init__(self):
        super().__init__('inventory_reporter')

        self._records = []
        self._expected_targets = []
        self._finished_targets = set()

        self.create_subscription(String, '/llm/scan_report',    self._report_cb,  10)
        self.create_subscription(String, '/llm/mission_command', self._cmd_cb,    10)
        self.create_subscription(String, '/mission_status_text', self._status_cb, 10)

        self.get_logger().info('inventory_reporter 시작')

    def _cmd_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            self._expected_targets = cmd.get('targets', [])
            self._finished_targets = set()
            self._records = []
            self.get_logger().info(f'새 미션 — 대상: {self._expected_targets}')
        except json.JSONDecodeError:
            pass

    def _report_cb(self, msg: String):
        try:
            record = json.loads(msg.data)
            self._records.append(record)
            self.get_logger().info(
                f"스캔 기록 추가: {record.get('item')} @ {record.get('location')}"
            )
        except json.JSONDecodeError:
            pass

    def _status_cb(self, msg: String):
        status = msg.data
        if status.startswith('MISSION_FINISHED'):
            wp = status.split(':')[1] if ':' in status else ''
            self._finished_targets.add(wp)

            if (self._expected_targets and
                    set(self._expected_targets).issubset(self._finished_targets)):
                self._finalize()

    def _finalize(self):
        if not self._records:
            self.get_logger().warn('저장할 스캔 데이터가 없습니다.')
            return

        _print_table(self._records)

        path = _save_csv(self._records)
        print(f'\n✅ CSV 저장 완료: {path}\n')
        self.get_logger().info(f'보고서 저장: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = InventoryReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
