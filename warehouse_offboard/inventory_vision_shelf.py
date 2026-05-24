import json
import time
from pathlib import Path
from typing import Optional, Tuple, List, Dict, Any

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode as zbar_decode
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def normalize_barcode_payload(payload: str) -> str:
    if payload is None:
        return ""
    return payload.strip().upper().replace(" ", "").replace("-", "_")


def load_inventory_db(path: str) -> Dict[str, Dict[str, Any]]:
    expanded = Path(path).expanduser()
    with open(expanded, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    raw_db = data.get("inventory_by_barcode", {})
    db = {}
    for barcode, item in raw_db.items():
        db[normalize_barcode_payload(barcode)] = item
    return db


class InventoryVisionShelf(Node):
    """
    QR-only inventory scanner.

    기존 구조:
      QR + shelf text OCR 동시 인식

    수정 후:
      QR만 인식해서 DB에서 물품 정보 조회
      shelf text는 Gazebo에 붙어 있지만 재고 판단에는 사용하지 않음
      Tesseract OCR 사용 안 함 → 딜레이 감소
    """

    def __init__(self):
        super().__init__("inventory_vision_shelf")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("inventory_db_path", "")
        self.declare_parameter("mission_target_topic", "/mission_target_name")
        self.declare_parameter("result_topic", "/inventory_scan_result")
        self.declare_parameter("debug_image_topic", "/inventory_debug_image")

        self.declare_parameter("scan_every_n_frames", 2)
        self.declare_parameter("duplicate_cooldown_sec", 2.0)
        self.declare_parameter("publish_debug_image", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.inventory_db_path = self.get_parameter("inventory_db_path").value
        self.mission_target_topic = self.get_parameter("mission_target_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.scan_every_n_frames = int(self.get_parameter("scan_every_n_frames").value)
        self.duplicate_cooldown_sec = float(self.get_parameter("duplicate_cooldown_sec").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        if not self.inventory_db_path:
            raise RuntimeError("inventory_db_path parameter is required")

        self.inventory_by_barcode = load_inventory_db(self.inventory_db_path)
        self.known_barcodes = set(self.inventory_by_barcode.keys())

        self.bridge = CvBridge()
        self.frame_count = 0
        self.current_target: Optional[str] = None
        self.last_partial_log_time = 0.0
        self.last_publish_time_by_key: Dict[str, float] = {}

        # 한 미션에서 같은 위치/같은 물품이 여러 번 출력되는 것 방지
        # 예: A-03-L2가 이미 출력되면, 같은 A-03-L2는 다시 publish하지 않음
        self.published_locations = set()

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(String, self.mission_target_topic, self.target_callback, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info("inventory_vision_shelf node started [QR-ONLY MODE]")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"known barcodes={sorted(self.known_barcodes)}")
        self.get_logger().info("Shelf text OCR is disabled. Inventory is determined from QR only.")

    def target_callback(self, msg: String):
        self.current_target = msg.data.strip().upper()

        # 새 선반 명령이 들어오면 이전 스캔 기록 초기화
        self.published_locations.clear()
        self.last_publish_time_by_key.clear()

        self.get_logger().info(f"Mission target received: {self.current_target}")
        self.get_logger().info("Inventory duplicate history cleared for new mission")

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % max(1, self.scan_every_n_frames) != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        barcodes, barcode_boxes = self.decode_barcodes_zbar(frame)

        known_barcode = self.select_center_barcode(frame, barcodes, barcode_boxes)

        item = None
        expected = None
        target_match = True

        if known_barcode:
            item = self.inventory_by_barcode.get(known_barcode)
            if item:
                expected = item.get("expected_location")

        # QR-only mode:
        # 위치 판정은 QR의 DB 위치를 기준으로 함.
        # shelf text OCR은 사용하지 않음.
        if self.current_target and expected:
            target_match = expected.startswith(self.current_target)

        if known_barcode and item:
            # 현재 목표 선반이 있으면, 해당 선반의 QR만 결과로 인정
            # 예: 목표가 A-03이면 A-03-L1~L4만 출력
            if self.current_target and expected:
                if not expected.startswith(self.current_target):
                    now = time.time()
                    if now - self.last_partial_log_time >= 3.0:
                        self.last_partial_log_time = now
                        self.get_logger().info(
                            f"Ignored QR outside current target | "
                            f"target={self.current_target}, barcode={known_barcode}, expected={expected}"
                        )
                    return

            # 한 미션에서 같은 위치는 한 번만 출력
            # 예: A-03-L2가 이미 출력됐으면 다시 출력하지 않음
            unique_key = expected if expected else known_barcode

            if unique_key in self.published_locations:
                if self.publish_debug_image:
                    self.publish_debug(
                        frame=frame,
                        barcodes=barcodes,
                        boxes=barcode_boxes,
                        known_barcode=known_barcode,
                        expected=expected,
                        target_match=target_match,
                        item=item,
                    )
                return

            self.published_locations.add(unique_key)

            self.publish_result(
                known_barcode=known_barcode,
                all_barcodes=barcodes,
                expected=expected,
                item=item,
                target_match=target_match,
            )
        else:
            now = time.time()
            if now - self.last_partial_log_time >= 3.0:
                self.last_partial_log_time = now
                self.get_logger().info(
                    f"Partial detection | known_barcode={known_barcode}, "
                    f"all_barcodes={barcodes}, waiting for known QR barcode."
                )

        if self.publish_debug_image:
            self.publish_debug(
                frame=frame,
                barcodes=barcodes,
                boxes=barcode_boxes,
                known_barcode=known_barcode,
                expected=expected,
                target_match=target_match,
                item=item,
            )

    def decode_barcodes_zbar(self, frame: np.ndarray) -> Tuple[List[str], List[Tuple[int, int, int, int]]]:
        """
        QR/barcode decoding using pyzbar/ZBar only.
        OpenCV QRCodeDetector is not used because QUIRC may not be linked.
        """
        variants = []

        for scale in [1.0, 1.5, 2.0, 3.0]:
            if scale == 1.0:
                resized = frame
            else:
                resized = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            blur = cv2.GaussianBlur(gray, (3, 3), 0)
            _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            adaptive = cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                5,
            )

            sharp = cv2.addWeighted(
                gray,
                1.8,
                cv2.GaussianBlur(gray, (0, 0), 2),
                -0.8,
                0,
            )

            variants.extend([
                (resized, scale),
                (gray, scale),
                (otsu, scale),
                (adaptive, scale),
                (sharp, scale),
            ])

        found = []
        boxes = []
        seen = set()

        for img, scale in variants:
            try:
                decoded = zbar_decode(img)
            except Exception:
                decoded = []

            for d in decoded:
                try:
                    payload = d.data.decode("utf-8", errors="ignore")
                except Exception:
                    payload = str(d.data)

                payload = normalize_barcode_payload(payload)
                if not payload:
                    continue

                if payload not in seen:
                    seen.add(payload)
                    found.append(payload)

                try:
                    r = d.rect
                    x = int(r.left / scale)
                    y = int(r.top / scale)
                    bw = int(r.width / scale)
                    bh = int(r.height / scale)
                    boxes.append((x, y, bw, bh))
                except Exception:
                    pass

        return found, boxes

    def select_center_barcode(self, frame: np.ndarray, barcodes: List[str], boxes: List[Tuple[int, int, int, int]]) -> Optional[str]:
        """여러 QR이 보일 때 화면 중앙에 가장 가까운 known QR을 선택한다."""
        if not barcodes:
            return None

        h, w = frame.shape[:2]
        img_cx = w / 2.0
        img_cy = h / 2.0

        candidates = []

        for i, code in enumerate(barcodes):
            if code not in self.known_barcodes:
                continue

            if i < len(boxes):
                x, y, bw, bh = boxes[i]
                cx = x + bw / 2.0
                cy = y + bh / 2.0
                area = bw * bh
                dist2 = (cx - img_cx) ** 2 + (cy - img_cy) ** 2
            else:
                area = 0
                dist2 = 1e18

            # 1순위: 화면 중앙에 가까운 QR
            # 2순위: 같은 거리라면 더 크게 보이는 QR
            candidates.append((dist2, -area, code))

        if not candidates:
            return None

        candidates.sort()
        return candidates[0][2]

    def publish_result(
        self,
        known_barcode: str,
        all_barcodes: List[str],
        expected: Optional[str],
        item: Dict[str, Any],
        target_match: bool,
    ):
        result = {
            "mode": "QR_ONLY",
            "barcode_detected": known_barcode,
            "all_barcodes": all_barcodes,
            "shelf_text_detected": None,
            "shelf_text_used": False,
            "expected_location_from_db": expected,
            "correct_location": True,
            "target_match": bool(target_match),
            "item_name": item.get("item_name"),
            "quantity": item.get("quantity"),
            "price": item.get("price"),
            "inbound_date": item.get("inbound_date"),
        }

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

        self.get_logger().info(
            "\n===== 재고 조사 결과 [QR 기준] =====\n"
            "판정: QR 기준 재고 확인 완료\n"
            f"박스 바코드: {known_barcode}\n"
            "선반 텍스트 인식: 사용 안 함\n"
            f"DB 지정 선반: {expected}\n"
            f"품명: {item.get('item_name')}\n"
            f"수량: {item.get('quantity')}\n"
            f"가격: {item.get('price')}\n"
            f"입고일: {item.get('inbound_date')}\n"
            f"명령 목표 선반 일치: {target_match}\n"
            "==================================="
        )

    def publish_debug(
        self,
        frame: np.ndarray,
        barcodes: List[str],
        boxes: List[Tuple[int, int, int, int]],
        known_barcode: Optional[str],
        expected: Optional[str],
        target_match: bool,
        item: Optional[Dict[str, Any]] = None,
    ):
        debug = frame.copy()

        for (x, y, w, h) in boxes:
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)

        lines = []
        if known_barcode and item:
            item_name = item.get('item_name', '')
            quantity = item.get('quantity', '')
            lines.append(f"Item: {item_name}")
            lines.append(f"Qty: {quantity}")
            lines.append(f"Match: {target_match}")
        elif known_barcode:
            lines.append(f"Barcode: {known_barcode}")

        y = 25
        for line in lines:
            cv2.putText(debug, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            y += 24

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"debug publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = InventoryVisionShelf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("inventory_vision_shelf stopped by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
