# !/bin/bash
# 노트북 코드를 Orin으로 쏘는 스크립트
echo "orin_firmware_bridge first_build Start ........"
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cd ~/tp_ws/orin_ws/
colcon build --packages-select mgs01_base_msgs
colcon build --packages-select carrier_teleop
source install/setup.bash
colcon build --packages-select orin_firmware_bridge --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "orin_firmware_bridge build End......"