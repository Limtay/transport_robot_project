 # 1. pygame 설치 (한 번만)
  # 주의: apt python3-pygame은 시스템 libsdl2(2.24+)와 버전이 안 맞아
  #       pygame.init()에서 "SDL compiled with ... linked to ..." 에러로 죽음.
  #       SDL을 자체 번들하는 pip 휠로 설치할 것.
  pip3 install --user --upgrade pygame

  # 2. 실행 (디스플레이 있는 노트북에서)
  cd ~/tp_ws/orin_ws
  source install/setup.bash
  ros2 run carrier_teleop keyboard_teleop

  기본값을 바꾸려면:
  ros2 run carrier_teleop keyboard_teleop --ros-args \
    -p max_linear:=0.5 -p max_angular:=0.5 \
    -p boost_factor:=3.0 -p boost_ramp_time:=1.0 \
    -p idle_timeout:=2.0 -p jeongae_pulse:=1.0
