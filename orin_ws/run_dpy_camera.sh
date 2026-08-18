#!/bin/bash
source /home/bridge/tp_ws/orin_ws/install/setup.bash

echo "Orin Environment: Arducam USB Camera (dpy_camera on-demand capture node) Start"
echo "----------------------------------------------------------------------"
echo " 노드가 대기 상태로 뜹니다. (평소 CPU ~0%, 명령 시에만 캡처)"
echo " 사진 캡처:  ros2 service call /dpy_camera/capture std_srvs/srv/Trigger {}"
echo "----------------------------------------------------------------------"

# install 폴더 대신, 원본이 있는 src 폴더의 스크립트/YAML 을 직접 실행 (빌드 필요 없음!)
python3 /home/bridge/tp_ws/orin_ws/src/dpy_camera/scripts/capture_node.py --ros-args \
    --params-file /home/bridge/tp_ws/orin_ws/src/dpy_camera/config/dpy_camera_params.yaml
