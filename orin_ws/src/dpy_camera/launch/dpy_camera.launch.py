import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     
    config_dir = os.path.join(
        get_package_share_directory('dpy_camera'),
        'config'
    )
    
    # 파라미터 파일 경로 지정
    params_file = os.path.join(config_dir, 'dpy_camera_params.yaml')

    # 저장 경로는 계정·머신마다 다르므로 yaml 에 박지 않고 여기서 홈 기준으로 만든다.
    save_dir = os.path.expanduser('~/dpy_camera_shots')

    # on-demand 캡처 노드 (평소 대기, 서비스 호출 시에만 캡처)
    capture_node = Node(
        package='dpy_camera',
        executable='capture_node',
        name='dpy_camera',
        namespace='', # 필요시 '/camera' 등으로 설정
        # 뒤에 오는 dict 가 yaml 값을 덮는다 — save_dir 만 런타임에 결정한다.
        parameters=[params_file, {'save_dir': save_dir}],
        output='screen'
    )

    return LaunchDescription([
        capture_node
    ])