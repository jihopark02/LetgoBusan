# 드론 기반 창고 자율 재고 관리 시스템

ROS2 + PX4 + Gazebo Harmonic 기반의 드론 자율 비행 및 자연어 명령 재고 관리 시뮬레이션 프로젝트입니다.

---

## 기술 스택

| 항목 | 내용 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 시뮬레이터 | Gazebo Harmonic (gz-transport13) |
| ROS-Gazebo 브릿지 | ros-humble-ros-gzharmonic |
| 드론 | PX4 SITL + px4vision (커스텀 카메라 모델) |
| 언어 | Python 3.10 |
| LLM | OpenAI gpt-4o-mini |
| QR 인식 | pyzbar |

> **주의**: `ros-humble-ros-gz-bridge` (ignition-transport11 기반)는 Gazebo Harmonic과 호환되지 않음.
> 반드시 `sudo apt install ros-humble-ros-gzharmonic` 으로 설치할 것.

---

## 프로젝트 구조

```
warehouse_offboard/
├── warehouse_offboard/
│   ├── goto_point.py              # 드론 미션 백엔드 (4층 스캔, 자율 비행)
│   ├── inventory_vision_shelf.py  # QR 기반 재고 인식 노드
│   ├── chat_mission_ui.py         # 자연어 채팅 UI (pygame)
│   ├── aruco_land.py              # ArUco 마커 정밀 착륙
│   ├── gz_camera_bridge.py        # Gazebo 카메라 브릿지 (전방 + 하단)
│   ├── llm_node.py                # GPT 자연어 미션 파싱
│   ├── mission_sequencer.py       # 다중 타겟 순차 미션 관리
│   ├── llm_scan_analyzer.py       # 스캔 결과 GPT 분석
│   ├── inventory_reporter.py      # 재고 보고서 출력 및 CSV 저장
│   └── result_report_node.py      # 최종 GPT 종합 보고서 (DB 대비 실측 비교)
├── models/
│   └── px4vision/
│       ├── model.config
│       └── model.sdf              # 전방·하단 카메라 추가된 커스텀 모델
├── params/
│   ├── goto_point.yaml            # 드론 미션 파라미터 (웨이포인트, 스캔 레이어)
│   ├── inventory_db_shelf.yaml    # 재고 DB (SKU → 품목·위치·수량)
│   └── inventory_vision_shelf.yaml
├── worlds/
│   └── warehouse.sdf              # Gazebo 창고 맵
└── setup.py
```

---

## 전체 데이터 흐름

```
[chat_mission_ui]
    ↓ /llm/user_input
[llm_node] ──→ /llm/response_text ──→ [chat_mission_ui 채팅창]
    ↓ /llm/mission_command
    ├──→ [mission_sequencer] ──→ /mission_target_name ──→ [goto_point] → 드론 비행
    ├──→ [llm_scan_analyzer]  (item_filter 수신)
    └──→ [result_report_node] (미션 정보 수신)

[inventory_vision_shelf] ──→ /inventory_scan_result ──→ [llm_scan_analyzer]
                         └──→ /inventory_debug_image  (바운딩박스 + 품명·수량 오버레이)

[llm_scan_analyzer] ──→ /llm/scan_report ──→ [inventory_reporter] → CSV 저장
                                          └──→ [result_report_node] → 최종 보고서
```

---

## 창고 구역 정보

| 구역 | 위치 | 품목 카테고리 |
|---|---|---|
| A-01 | 우측 하단 | 과일 (grape, apple, strawberry, watermelon) |
| A-02 | 좌측 하단 | 식품 (ramen, canned food, water bottle, snack) |
| A-03 | 우측 상단 | 의류 (t-shirt, jeans, sneakers, jacket) |
| A-04 | 좌측 상단 | 전자기기 (wireless earbuds, smartwatch, bluetooth speaker, power bank) |

각 구역은 L1(1층) ~ L4(4층) 선반으로 구성되며, 선반마다 QR코드가 부착되어 있음.

---

## 드론 미션 파이프라인

```
WAIT_HOME → TAKEOFF → YAW_ALIGN → MOVE_GLOBAL_Y → MOVE_GLOBAL_X
→ SCAN_LAYER (L1 → L2 → L3 → L4)
→ RETURN_GLOBAL_X → RETURN_GLOBAL_Y → PRELAND_YAW_HOME
→ PRELAND_SETTLE → WAIT_ARUCO_LAND → FINISHED
```

---

## 실행 방법

### 사전 준비

```bash
# 환경 정리
pkill -9 -f px4
pkill -9 -f "gz sim"
pkill -9 -f MicroXRCEAgent
pkill -9 -f ros_gz_bridge
rm -rf /tmp/px4* ~/.ros/log ~/.gz/rendering ~/.cache/gazebo

export OPENAI_API_KEY=your_api_key_here
```

### Terminal 1: Gazebo

```bash
cd ~/capstone_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
# 커스텀 모델(카메라 포함 px4vision)을 기본 PX4 모델보다 먼저 탐색하도록 설정
export GZ_SIM_RESOURCE_PATH=$HOME/capstone_ws/src/warehouse_offboard/models:$GZ_SIM_RESOURCE_PATH:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
gz sim -r ~/capstone_ws/src/warehouse_offboard/worlds/warehouse.sdf
```

### Terminal 2: PX4

```bash
cd ~/PX4-Autopilot && source ~/.bashrc
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4006 \
PX4_SIM_MODEL=gz_px4vision \
PX4_GZ_MODEL_POSE="2.1,-1.5,0.3,0,0,0" \
./build/px4_sitl_default/bin/px4
```

### Terminal 3: MicroXRCEAgent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 4: QGroundControl

```bash
cd ~/Downloads && ./QGroundControl.AppImage
```

### Terminal 5: Gazebo 카메라 bridge

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard gz_camera_bridge
```

브릿지 토픽:
- `/camera/image_raw` — 전방 카메라
- `/camera/down_image_raw` — 하단 카메라

### Terminal 6: 재고 인식 노드

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard inventory_vision_shelf \
  --ros-args \
  --params-file $HOME/capstone_ws/src/warehouse_offboard/params/inventory_vision_shelf.yaml \
  -p image_topic:=/camera/image_raw \
  -p inventory_db_path:=$HOME/capstone_ws/src/warehouse_offboard/params/inventory_db_shelf.yaml \
  -p require_target_match:=false
```

디버그 이미지: `/inventory_debug_image` (바운딩박스 + 품명·수량 표시)

### Terminal 7: goto_point (드론 미션 백엔드)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard goto_point \
  --ros-args --params-file ~/capstone_ws/src/warehouse_offboard/params/goto_point.yaml
```

### Terminal 8: aruco_land (정밀 착륙)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard aruco_land \
  --ros-args --params-file ~/capstone_ws/src/warehouse_offboard/params/goto_point.yaml
```

### Terminal 9: llm_node (GPT 미션 파싱)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export OPENAI_API_KEY=your_api_key_here
ros2 run warehouse_offboard llm_node
```

### Terminal 10: mission_sequencer (순차 미션)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard mission_sequencer
```

### Terminal 11: llm_scan_analyzer (스캔 결과 분석)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export OPENAI_API_KEY=your_api_key_here
ros2 run warehouse_offboard llm_scan_analyzer
```

### Terminal 12: inventory_reporter (보고서 저장)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard inventory_reporter
```

### Terminal 13: result_report_node (최종 종합 보고서)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export OPENAI_API_KEY=your_api_key_here
ros2 run warehouse_offboard result_report_node
```

### Terminal 14: chat_mission_ui (자연어 명령 UI)

```bash
cd ~/capstone_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run warehouse_offboard chat_mission_ui
```

### Terminal 15: rqt_image_view (카메라 디버그)

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
# 토픽: /inventory_debug_image (인식 결과 오버레이)
# 토픽: /camera/image_raw (원본)
```

---

## 자연어 명령 예시

| 입력 | 동작 |
|---|---|
| `전체 창고 재고 조사해줘` | A-01 ~ A-04 순차 스캔 |
| `우측 선반만 확인해줘` | A-01, A-03 순차 스캔 |
| `2번 구역 가봐` | A-02로 이동 (스캔 없음) |
| `grape 재고 확인해줘` | A-01만 스캔 (item_filter 적용) |
| `지금 어디야?` | 드론 현재 위치·상태 응답 |

---

## 주요 ROS2 토픽

| 토픽 | 발행자 | 구독자 | 내용 |
|---|---|---|---|
| `/llm/user_input` | chat_mission_ui | llm_node | 자연어 입력 |
| `/llm/mission_command` | llm_node | mission_sequencer, llm_scan_analyzer, result_report_node | GPT 파싱 결과 (JSON) |
| `/llm/response_text` | llm_node | chat_mission_ui | GPT 응답 텍스트 |
| `/mission_target_name` | mission_sequencer | goto_point | 드론 타겟 구역 |
| `/mission_status_text` | goto_point | 전체 | 미션 상태 |
| `/inventory_scan_result` | inventory_vision_shelf | llm_scan_analyzer | QR 스캔 원시 결과 |
| `/inventory_debug_image` | inventory_vision_shelf | rqt_image_view | 바운딩박스 디버그 이미지 |
| `/llm/scan_report` | llm_scan_analyzer | inventory_reporter, result_report_node | GPT 분석 포함 스캔 레코드 |
| `/llm/scan_analysis` | llm_scan_analyzer | - | GPT 분석 텍스트 |
| `/llm/final_report` | result_report_node | llm_node | 최종 종합 보고서 |
| `/camera/image_raw` | gz_camera_bridge | inventory_vision_shelf | 전방 카메라 |
| `/camera/down_image_raw` | gz_camera_bridge | aruco_land | 하단 카메라 |
