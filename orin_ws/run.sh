#!/bin/bash
# 노트북 코드를 Orin으로 쏘는 스크립트
echo "orin_firmware_bridge comm_test_node Start...."
# colcon build --packages-select orin_firmware_bridge --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
sudo bash -c "source /opt/ros/humble/setup.bash && source /home/limtay/tp_ws/orin_ws/install/setup.bash && ros2 run orin_firmware_bridge comm_test_node"