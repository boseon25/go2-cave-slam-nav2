# 프로젝트 요약 — 동굴 재난 탐사 및 인명·위험물 탐지 로봇

> 실행 방법은 [README.md](README.md)를 참고하세요. 이 문서는 폴더 구조와 각 파일이
> 하는 역할, 지금까지의 개발 히스토리를 정리한 요약 문서입니다.

## 1. 한 줄 요약

Unitree Go2(사족보행 로봇)를 Gazebo Classic + ROS 2 Humble로 시뮬레이션하여, DARPA
SubT 스타일 동굴에서 SLAM으로 지도를 만들고, Nav2로 자율주행하며, YOLO로 카메라
영상에서 조난자(사람)를 탐지하고 그 위치를 map 좌표로 계산해 보고하는 캡스톤
프로젝트입니다.

## 2. 폴더 구조

```
capstone2/
├── README.md                    # 실행 방법 (터미널별 명령어, 트러블슈팅)
├── PROJECT_SUMMARY.md           # 이 문서
├── gazebo_cave_world/           # 동굴 맵 (LTU-RAI 오픈소스, 서브모듈 아님, 직접 포함)
│   └── worlds/cave_world_custom.world   # 실제 사용하는 커스텀 동굴 월드
└── go2_ws/                      # ROS 2 colcon 워크스페이스
    ├── yolo11n.pt                # YOLO 모델 가중치 (최초 실행 시 자동 다운로드, git 추적 안 함)
    └── src/
        ├── unitree-go2-ros2/     # Go2 로봇 관련 패키지 (아래 3.1 참고)
        └── m-explore-ros2/       # frontier exploration 패키지 (아직 실사용 불가, 3.4 참고)
```

## 3. 핵심 구성 요소

### 3.1 로봇 (`go2_ws/src/unitree-go2-ros2/`)

- **`champ/`**: [CHAMP](https://github.com/chvmp/champ) 사족보행 제어 프레임워크 (다리 역기구학, 걸음걸이 생성, SLAM/Nav2 launch 템플릿의 원본).
- **`robots/descriptions/go2_description/`**: Go2 로봇의 URDF/xacro. 센서는 `xacro/` 아래:
  - `laser.xacro` — 전방 2D LiDAR (`front_laser`, 사거리 20m로 확장)
  - `lidar3d.xacro` — 16채널 3D LiDAR (VLP-16 스타일, `libgazebo_ros_ray_sensor.so` 재사용, 오늘 세션에서 코스트맵 바닥 오인식 버그의 원인이었던 센서)
  - `gazebo.xacro` — 발 마찰(mu1/mu2), 물리 파라미터
  - `robot.xacro` — 위 센서들 + `champ_description/asus_camera.urdf.xacro`(RGB-D 카메라) 조립
- **`robots/configs/go2_config/`**: 이 프로젝트의 실질적인 "메인" 패키지. launch 파일, 튜닝된 파라미터, 맵, 커스텀 스크립트가 모두 여기 있습니다.
  - `launch/gazebo.launch.py` — Gazebo + Go2 스폰
  - `launch/slam.launch.py` — 라이브 SLAM(slam_toolbox) + Nav2 + RViz
  - `launch/view_map.launch.py` — **저장된 지도**(`cave_world_map.yaml`)를 불러와 AMCL + Nav2 + RViz로 자율주행 (slam.launch.py와 독립적)
  - `launch/navigate.launch.py`, `octomap.launch.py`, `bringup.launch.py` — 위 두 launch가 내부적으로 조합해 쓰는 부품들
  - `config/gait/gait.yaml` — 걸음걸이 파라미터 (스윙 높이, 속도 등; 몸통 흔들림 안정화 튜닝 완료)
  - `config/ros_control/ros_control.yaml` — 관절 PID 게인
  - `config/autonomy/navigation.yaml` — Nav2 전체 설정 (AMCL, costmap, 코스트맵 관측 소스 등)
  - `config/autonomy/slam.yaml`, `octomap.yaml` — SLAM / 3D octomap 설정 (octomap은 아직 메인 파이프라인에 연결 전)
  - `maps/cave_world_map.pgm/.yaml` — teleop으로 수동 매핑해서 저장해둔 동굴 지도
  - `scripts/yolo_survivor_detector.py` — **YOLO 기반 조난자 탐지 노드** (아래 3.3 참고)
  - `scripts/ground_truth_odom_bridge.py`는 `champ_base/scripts/`에 있음 (아래 3.2 참고)

### 3.2 odometry: ground truth 우회 (`champ_base/scripts/ground_truth_odom_bridge.py`)

CHAMP의 다리 기구학 기반 오도메트리가 이 Go2 포팅에서 발산하는 버그가 있어, 대신
Gazebo의 실제(ground truth) pose를 그대로 `odom` 프레임으로 발행합니다. 시뮬레이션
전용 우회책이며, 몸통이 걸을 때 흔들리는 걸 완화하기 위한 지수이동평균(EMA) 필터도
포함되어 있습니다. **실제 로봇에는 쓸 수 없습니다.**

이 덕분에 `odom` 프레임이 사실상 Gazebo world(=map) 좌표계와 같아서, `view_map.launch.py`의
AMCL과 YOLO 탐지 노드의 좌표 계산이 모두 이 사실에 의존하고 있습니다.

### 3.3 비전: YOLO 조난자 탐지 (`go2_config/scripts/yolo_survivor_detector.py`)

[THENAEUN/go2-cave-survivor-detection](https://github.com/THENAEUN/go2-cave-survivor-detection)의
접근법을 이 프로젝트의 카메라 토픽에 맞게 이식한 것입니다.

- `/camera/image_raw`(RGB) → YOLO(`yolo11n.pt`)로 사람(person) 탐지
- `/camera/depth/image_raw` → 바운딩박스 중심 영역의 depth 중앙값으로 거리 추정
- 카메라 intrinsic(`/camera/depth/camera_info`) + TF(base_link↔카메라) + 로봇의 world 위치(`/odom/ground_truth`)로 **조난자의 map 좌표**를 계산해 `/survivor_position`(PointStamped)로 발행
- 3프레임 연속 탐지 + 1m 이내 접근 시 `/cmd_vel`로 로봇 정지
- `cv_bridge`가 이 환경의 NumPy 2.x와 ABI 충돌이 나서, cv_bridge 없이 raw image bytes를 numpy로 직접 파싱하도록 수정됨. GPU 추론도 cuDNN 버전 불일치로 실패해 CPU 고정.

### 3.4 아직 안 되는 것: frontier exploration (`m-explore-ros2`)

완전 자율 SLAM 탐사(사람이 조종 안 해도 알아서 돌아다니며 매핑)를 위해 시도했지만,
explore_lite가 같은 위치에서 목표를 계속 "도달"로 잘못 판단해 제자리에서 맴도는 버그가
있어 실사용을 포기했습니다. 현재 지도는 teleop(수동 조종)으로 만든 것입니다.

## 4. 개발 히스토리 (git log 기준)

| 날짜 | 커밋 | 내용 |
|---|---|---|
| 07-23 | `28a2ac3` | 최초 캡스톤 구성: Go2 + Gazebo + SLAM + Nav2 + 카메라 |
| 07-23 | `2afa06c` | 바위 등반을 위해 관절 토크/PID, 스윙 높이 상향 |
| 07-24 | `2c62928` | 커스텀 동굴 월드 추가, 기본 월드로 전환 |
| 07-24 | `1758618` | 몸통 흔들림 안정화(스윙 높이/속도/PID 재조정), 3D LiDAR·octomap 기초 작업, **YOLO 조난자 탐지 최초 도입** |
| 07-25 | `9c85c30` | `view_map.launch.py` 추가 (저장된 지도를 RViz에 표시) |
| 07-25 | `6c2d67b` | 코스트맵이 바닥을 장애물로 오인식하던 버그 수정 (`min_obstacle_height` 0.0→0.08) |
| 07-25 | `11383e2` | AMCL 오도메트리 신뢰도 상향 (`alpha1~5` 0.2→0.02)로 위치 드리프트 완화 |
| 07-25 | `10c7903` | README에 `view_map.launch.py`/YOLO 노드 실행법 문서화 |
| 07-25 | `e58408b` | YOLO 탐지에 **조난자 world 좌표 계산·발행** 기능 추가 |

## 5. 이번 세션에서 발견/해결한 주요 이슈

1. **몸통 흔들림 → SLAM 지도 흔들림**: 걸음걸이의 스윙 높이/속도, 관절 PID, 발 마찰,
   ground truth odom의 EMA 필터를 함께 조정해서 완화.
2. **로컬/글로벌 코스트맵이 로봇 주변을 온통 장애물로 인식**: 3D LiDAR 최하단
   채널이 로봇 앞 바닥(약 1.3m 지점)을 찍는데 `min_obstacle_height: 0.0`이라 그
   바닥 반사점이 전부 장애물로 마킹됨 → `0.08`로 수정.
3. **RViz가 보여주는 위치와 Gazebo 실제 위치가 다름**: AMCL이 저장된 지도의
   미확인(unknown) 구간에서 스캔 매칭에 실패해 위치 추정이 실제 위치에서 최대
   수 미터씩 벗어남. `ground_truth_odom_bridge` 덕분에 오도메트리 자체는 거의
   드리프트가 없다는 점을 이용해 AMCL의 오도메트리 노이즈 모델(`alpha1~5`)을
   대폭 낮춰서 해결. (AMCL을 완전히 없애고 map→odom을 고정 변환으로 대체하는
   방법도 시도했지만, 그러면 "2D Pose Estimate" 같은 표준 워크플로우가 아예
   동작하지 않게 되어 되돌림.)
4. **RViz "2D Pose Estimate"가 이상하게 동작**: 버그가 아니라, 클릭만 하고
   드래그하지 않으면 방향(yaw)이 로봇의 실제 방향과 무관하게 설정되는 것이었음.
5. **launch 파일이 커스텀 Nav2 파라미터를 안 씀**: `view_map.launch.py` 초기
   버전이 `params_file` 인자를 안 넘겨서 Nav2 기본값(turtlebot3용)으로 조용히
   폴백되고 있었음 — 커스텀 코스트맵 튜닝이 전혀 적용되지 않던 원인.
6. **중복 프로세스 누적**: Gazebo/Nav2 스택을 여러 번 재시작하면서 이전 프로세스가
   완전히 안 죽고 남아 새 인스턴스와 충돌(노드 이름 중복 등)을 일으킨 사례가
   여러 번 있었음 — 재시작 시 관련 프로세스를 빠짐없이 종료하는 것이 중요.

## 6. 현재 상태

- 라이브 SLAM(`slam.launch.py`)과 저장된 지도 기반 자율주행(`view_map.launch.py`)
  둘 다 정상 동작 확인.
- YOLO 조난자 탐지 + map 좌표 계산 + 정지 로직 정상 동작 확인 (테스트 모형으로 검증).
- Frontier exploration은 버그로 보류, teleop으로 수동 매핑 중.
- 3D LiDAR/octomap은 코스트맵 관측 소스로는 쓰이고 있지만, 별도 3D 지도 시각화(octomap_server)는 아직 메인 파이프라인에 안 붙어 있음.

## 7. 앞으로 할 만한 것

- explore_lite 버그 원인 파악 후 완전 자율 매핑 재시도
- 조난자 위치까지 다중 경로계획 알고리즘(A*/Theta*/RRT*) 비교
- octomap 기반 3D 지도 시각화를 메인 파이프라인에 연결
