# 동굴 재난 탐사 및 인명·위험물 탐지 로봇 (Capstone)

Unitree Go2(사족보행 로봇)를 Gazebo Classic(ROS 2 Humble) 위에서 시뮬레이션하여,
동굴(DARPA SubT 스타일) 환경에서 SLAM으로 지도를 작성하고 Nav2로 자율주행하며,
카메라로 조난자·위험물을 탐지하는 프로젝트입니다.

- **맵**: [LTU-RAI/gazebo_cave_world](https://github.com/LTU-RAI/gazebo_cave_world)
- **로봇**: Unitree Go2 (다리 제어: [CHAMP](https://github.com/chvmp/champ) 프레임워크, [anujjain-dev/unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) 기반)
- **SLAM**: slam_toolbox (online async)
- **자율주행**: Nav2
- **센서**: 2D LiDAR(front_laser) + RGB-D 카메라(camera)

## 사전 준비물

- Ubuntu 22.04
- ROS 2 Humble (Desktop 설치 권장)
- Gazebo 11 (Gazebo Classic)

## 1. 최초 설치 (한 번만)

```bash
# 1) 이 저장소를 정확히 이 경로에 클론 (launch 파일 기본값이 이 경로를 가정함)
cd ~
git clone --recursive <이 저장소 URL> capstone2
cd ~/capstone2

# 2) ROS 2 의존 패키지 설치
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-localization \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rqt-image-view \
  python3-pyqt5

# 3) go2_ws 워크스페이스 빌드
cd ~/capstone2/go2_ws
source /opt/ros/humble/setup.bash
colcon build
```

> `--recursive`로 클론하지 못했다면 `git submodule update --init --recursive`를 실행하세요.
> `gazebo_cave_world`와 `go2_ws`는 반드시 `~/capstone2` 바로 아래, 같은 depth에 있어야 합니다
> (launch 파일 기본 경로가 `~/capstone2/gazebo_cave_world/...`를 가정하고 있음).

## 2. 실행 방법 — 터미널 3개

매 터미널마다 아래 두 줄로 환경을 먼저 불러오세요:
```bash
source /opt/ros/humble/setup.bash
source ~/capstone2/go2_ws/install/setup.bash
```

### 터미널 1 — Gazebo + Go2 로봇 (동굴 맵, 터널 입구 앞에 스폰)

```bash
ros2 launch go2_config gazebo.launch.py
```

- 인자 없이 실행하면 자동으로 동굴 맵 + 터널 입구 앞 평지에서 로봇이 스폰됩니다.
- Gazebo 창이 뜨고 15초 뒤 로봇이 스폰됩니다 (지형 로딩 시간 확보용 지연).
- Gazebo ground-truth 기반 odometry 브리지도 이 안에서 자동으로 함께 실행됩니다.

### 터미널 2 — SLAM + Nav2 + RViz

```bash
ros2 launch go2_config slam.launch.py sim:=true rviz:=true
```

- RViz가 뜨면서 실시간으로 지도가 그려집니다.
- **RViz2가 뜨자마자 죽는다면** (`libpthread` 관련 symbol lookup error): VSCode 등 snap 환경에서 `GTK_PATH` 환경변수가 원인입니다. 아래처럼 실행하세요.
  ```bash
  env -u GTK_PATH ros2 launch go2_config slam.launch.py sim:=true rviz:=true
  ```

### 터미널 3 — 로봇 조종 (지도를 넓히려면 로봇이 움직여야 합니다)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- **이 터미널 창을 클릭해서 포커스를 준 상태에서** `i`(전진) / `,`(후진) / `j`,`l`(회전) 키를 누르세요.
- 로봇은 터널을 바라보고 스폰되므로 `i`를 누르면 바로 동굴 안으로 걸어 들어갑니다.

### 카메라 영상 보기 (선택)

RViz 왼쪽 아래 **Add → By display type → Image** 추가 후, Topic을 `/camera/image_raw`로 설정하면
로봇 시점 카메라 영상이 보입니다. (RViz의 "Camera" 디스플레이 타입이 아니라 **"Image"** 타입을 써야
순수 카메라 영상만 나옵니다 — "Camera" 타입은 RViz 3D 씬이 같이 합성되어 나옵니다.)

또는 별도 창으로:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### 터미널 2 대안 — 이미 만든 지도 불러오기 (라이브 SLAM 대신)

새로 매핑하지 않고 저장해둔 `go2_config/maps/cave_world_map.yaml`을 바로 불러와서
AMCL로 위치를 추정하며 Nav2로 자율주행하고 싶다면:

```bash
ros2 launch go2_config view_map.launch.py
```

- `ros2 launch go2_config slam.launch.py ...`(터미널 2, 라이브 SLAM)와는 완전히 독립적인 명령이라 서로 바꿔 써도 되고, 동시에 켜지만 않으면 됩니다 (둘 다 `/map` 토픽에 발행해서 충돌납니다).
- RViz에서 **2D Pose Estimate**로 초기 위치를 다시 잡을 때는 클릭한 채로 로봇이 실제로 향한 방향까지 드래그한 뒤 놓으세요. 그냥 클릭만 하면 방향이 (보통 0°로) 엉뚱하게 설정되어 로봇이 반대 방향으로 움직이는 것처럼 보입니다.
- **Nav2 Goal**로 지도 위 목표 지점을 찍으면 경로가 계획되고 로봇이 자율주행합니다.

### 터미널 4 — YOLO 조난자(사람) 탐지 (선택)

```bash
# 최초 1회만: ultralytics/torch 설치
# (cv_bridge가 시스템 numpy(1.x)에 맞춰 컴파일되어 있어서, 가상환경 등 다른 numpy를
#  쓰는 python3로 설치/실행하면 numpy 2.x ABI 충돌로 크래시합니다. 반드시 시스템 python3 사용)
/usr/bin/python3 -m pip install --user ultralytics
/usr/bin/python3 -m pip install --user -U matplotlib   # ultralytics가 끌고 오는 numpy 2.x와의 충돌 방지

/usr/bin/python3 ~/capstone2/go2_ws/src/unitree-go2-ros2/robots/configs/go2_config/scripts/yolo_survivor_detector.py --ros-args -p use_sim_time:=true
```

- `/camera/image_raw`, `/camera/depth/image_raw`를 구독해 사람(조난자)을 탐지하고, 3프레임 연속 탐지 + 1m 이내로 접근하면 `/cmd_vel`로 로봇을 정지시킵니다.
- 실행하면 별도의 **"Go2 Survivor Detection"** 창에 바운딩박스·신뢰도·추정 거리가 표시됩니다 (rqt_image_view는 원본 영상만 보여줄 뿐 판단은 하지 않습니다 — 판단 결과는 이 창에서 확인).
- YOLO 모델(`yolo11n.pt`)은 최초 실행 시 자동 다운로드됩니다.

#### 조난자 좌표(map 기준) 확인하기

3프레임 연속 탐지되면 카메라 픽셀+depth를 카메라 intrinsic과 TF(`base_link` ↔ 카메라 optical frame),
그리고 `/odom/ground_truth`(로봇의 map 기준 실제 위치)를 이용해 조난자의 **map 좌표계 3D 좌표**를 계산합니다.

- **터미널 4 화면 자체**에 약 1초마다 로그로 바로 찍힙니다 (별도 설정 필요 없음):
  ```
  [INFO] [yolo_survivor_detector]: 조난자 탐지 중: 인원=1, 가장 가까운 거리=1.28 m, 연속 탐지=64/3, 로봇 좌표=(5.91, 3.87, 0.22), 조난자 좌표=(5.46, 5.51, 0.30)
  ```
- 로그 텍스트 말고 좌표값만 순수하게 보고 싶다면, **터미널 5**를 새로 열어서:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /survivor_position
  ```
  를 실행하면 `geometry_msgs/PointStamped`(`header.frame_id: map`, `point.x/y/z`)로 실시간 발행됩니다.
- 비전 기반 추정이라 실제 조난자 모형(rescue_randy)의 중심점과는 보통 수십 cm ~ 1m 정도 오차가 있을 수 있습니다 (모형이 눕거나 기댄 자세일 때 특히 그렇습니다).

## 3. 프로세스 다 껐다가 다시 이어보고 싶을 때

Gazebo/RViz 창만 닫았다면 (`gzserver`, SLAM, Nav2 노드는 백그라운드에서 계속 실행 중) 창만 다시 붙이면 됩니다:
```bash
ros2 launch gazebo_ros gzclient.launch.py
env -u GTK_PATH rviz2 -d ~/capstone2/go2_ws/install/champ_navigation/share/champ_navigation/rviz/slam.rviz --ros-args -p use_sim_time:=true
```

전체를 처음부터 다시 시작하려면 터미널 1, 2를 Ctrl+C로 끄고 다시 실행하세요.

## 주요 토픽

| 토픽 | 설명 |
|---|---|
| `/scan` | 2D LiDAR 스캔 |
| `/camera/image_raw` | RGB 카메라 (YOLO 등 비전 인식용) |
| `/camera/depth/image_raw`, `/camera/points` | Depth 이미지 / 포인트클라우드 (3D 위치 추정용) |
| `/map` | slam_toolbox가 생성 중인 점유격자지도 |
| `/cmd_vel` | 로봇 이동 명령 |
| `/odom` | 로봇 odometry (Gazebo ground-truth 기반) |

## 알려진 이슈 / 트러블슈팅

- **로봇이 스폰 직후 넘어짐**: 스폰 위치가 지형(바위) 안이거나 너무 높은 곳이면 발생합니다. 기본값(터널 입구 앞 평지)을 그대로 쓰면 안정적으로 섭니다.
- **`/map`이 계속 malformed / 0x0으로 나옴**: `/odom/raw`(CHAMP 자체 다리 기구학 오도메트리)가 수치적으로 발산하는 버그가 있어, `champ_bringup`에서 기본 EKF 체인 대신 Gazebo ground-truth 기반 `ground_truth_odom_bridge` 노드를 쓰도록 이미 패치되어 있습니다 (`champ_base/scripts/ground_truth_odom_bridge.py`). 실제 로봇(하드웨어)에는 쓸 수 없는 시뮬레이션 전용 우회책이니 참고하세요.
- **라이다가 전부 `inf`만 찍힘**: 라이다 최대 사거리를 3.5m → 20m로 늘려뒀습니다 (`go2_description/xacro/laser.xacro`). 개활지에서는 3.5m로는 아무것도 안 보입니다.
- **`contact_sensor` 프로세스가 CPU를 계속 잡아먹음**: 알려진 CHAMP 이슈라 launch에서 비활성화해뒀습니다.
- **로컬/글로벌 코스트맵이 로봇 주변을 온통 장애물로 인식**: 3D 라이다(`lidar3d`)의 최하단 채널이 로봇 앞 바닥을 찍는데, `min_obstacle_height`가 0.0이면 그 바닥 반사점이 전부 장애물로 마킹됩니다 (`navigation.yaml`에서 0.08로 수정 완료). 코스트맵 관련 파라미터를 또 만지게 되면 이 값부터 확인하세요.
- **AMCL 위치 추정이 실제 위치에서 크게 벗어남**: `ground_truth_odom_bridge`가 Gazebo ground truth를 그대로 odom으로 쓰기 때문에 실제로는 오도메트리가 거의 드리프트하지 않는데, AMCL 기본 파라미터(`alpha1~5`)는 실제 로봇의 (드리프트가 있는) 오도메트리를 가정하고 있어 저장된 맵의 미확인 구간에서 위치 추정이 수 미터씩 벗어날 수 있었습니다. `alpha1~5`를 0.02로 낮춰 오도메트리를 훨씬 더 신뢰하도록 튜닝해뒀습니다.
- **RViz "2D Pose Estimate"가 이상하게 동작**: 클릭만 하고 드래그하지 않으면 방향(yaw)이 로봇의 실제 방향과 무관하게 설정됩니다. 클릭한 채로 로봇이 실제로 향한 방향까지 드래그했다가 놓으세요.

## 향후 계획

- `m-explore-ros2` (frontier exploration)로 완전 자율 SLAM 탐사: 워크스페이스에 빌드는 되어 있으나, explore_lite가 같은 위치에서 목표를 계속 "도달"로 잘못 판단해 제자리에서 맴도는 버그가 있어 아직 실사용 불가 (현재는 teleop으로 수동 매핑)
- 조난자 위치까지 다중 경로계획 알고리즘(A*/Theta*/RRT* 등) 비교
