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

## 향후 계획

- `m-explore-ros2` (frontier exploration)로 완전 자율 SLAM 탐사 (워크스페이스에 클론만 되어 있고 아직 빌드/연동 전)
- YOLO 기반 조난자/위험물 탐지 + 3D 위치 추정 후 지도 위 좌표 보고
- 조난자 위치까지 다중 경로계획 알고리즘(A*/Theta*/RRT* 등) 비교
