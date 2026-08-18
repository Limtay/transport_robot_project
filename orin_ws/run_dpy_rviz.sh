#!/bin/bash
# dpy_camera 가 찍은 사진을 RViz2 로 본다. (run_dpy_camera.sh 와 짝)
#
# 사진 토픽은 TRANSIENT_LOCAL(래치)이라 rqt_image_view 로는 안 보인다 —
# QoS 를 지정할 수 있는 RViz2 + 아래 설정 파일이 필요하다. (CAMERA_ACTION.md §14)

CONFIG=/home/bridge/tp_ws/orin_ws/src/dpy_camera/config/dpy_camera.rviz

source /opt/ros/humble/setup.bash
# install 이 아직 없어도(빌드 전) 토픽만 보면 되므로 실패해도 넘어간다.
source /home/bridge/tp_ws/orin_ws/install/setup.bash 2>/dev/null

echo "Orin Environment: dpy_camera 촬영 결과 뷰어 (RViz2) Start"
echo "----------------------------------------------------------------------"
echo " 왼쪽 Displays 패널 > Shot > Topic > Value 의 끝 숫자만 바꿔 장을 넘긴다."
echo "   /dpy_camera/image_raw_1  →  _2  →  _3 ...   (1-base, num_shots 만큼)"
echo " 래치 토픽이라 촬영이 한참 지난 뒤에 띄워도 사진이 나온다."
echo " Fixed Frame(TF) 경고는 3D 뷰를 안 쓰므로 무시해도 된다."
echo "----------------------------------------------------------------------"

# 원본이 있는 src 폴더의 설정을 직접 읽는다 (빌드 필요 없음 — run_dpy_camera.sh 와 동일)
exec rviz2 -d "$CONFIG"
