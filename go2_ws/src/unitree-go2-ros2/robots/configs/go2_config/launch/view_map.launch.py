from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('go2_config')

    default_map_path = PathJoinSubstitution(
        [this_package, 'maps', 'cave_world_map.yaml']
    )

    navigate_launch_path = PathJoinSubstitution(
        [this_package, 'launch', 'navigate.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='RViz/Nav2에 띄울 저장된 맵(.yaml) 경로'
        ),

        DeclareLaunchArgument(
            name='sim',
            default_value='true',
            description='Enable use_sim_time to true'
        ),

        # navigate.launch.py(map_server + AMCL + Nav2)를 저장된 맵으로 재사용한다.
        # RViz는 champ_navigation/navigation.rviz를 쓰는데, 이 설정은 slam.rviz와
        # 달리 Map 디스플레이가 map_server의 Transient Local QoS와 이미 맞게
        # 되어 있어 지도가 바로 뜨고 2D Goal Pose로 경로 계획도 보인다.
        #
        # AMCL을 완전히 빼고 map->odom을 고정 변환으로 대체하는 방법도 시도했지만
        # (odom이 실제로는 ground_truth_odom_bridge를 통한 Gazebo ground truth라
        # map과 이미 같은 좌표계라서), 그러면 2D Pose Estimate 같은 표준 RViz/Nav2
        # 워크플로우가 아예 동작하지 않게 되어 되돌렸다. 대신 navigation.yaml의
        # AMCL alpha1~5(오도메트리 노이즈)를 크게 낮춰서, 저장된 맵의 미확인 구간
        # 에서도 실제 위치에서 크게 드리프트하지 않도록 했다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigate_launch_path),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'sim': LaunchConfiguration('sim'),
                'rviz': 'true',
            }.items(),
        ),
    ])
