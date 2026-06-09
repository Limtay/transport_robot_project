# USB 연결 ping 확인 
ping 192.168.55.1

# 실행 권한(Execute Permission)' 부여
chmod +x deploy.sh

# orin에 Data 이동
cd ~/orin_ws
./deploy.sh

# orin ssh 접속
# Orin case 바꿔야함, 10번 포트안열려있음.
ssh swarm@192.168.55.1
ssh swarm@10.108.169.214

colcon build --symlink-install

# Plotting 하기 
ros2 run plotjuggler plotjuggler

ros2 bag record -e "/carrier/.*" /carrier_camera/compressed 

ros2 run orin_firmware_bridge comm_test_node --ros-args \
    -p enable_infra1:=false \
    -p enable_infra2:=false