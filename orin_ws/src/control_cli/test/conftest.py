"""테스트 프로세스를 독립 DDS 도메인에 가둔다.

CLI e2e 의 가짜 bridge 는 실제 bridge 와 같은 토픽·서비스 이름을 쓴다. colcon test 가
패키지를 병렬로 돌리면 orin_firmware_bridge 의 C++ 테스트와 서로를 발견해 요청이 엉뚱한
상대에게 간다 (전체 실행 시 실제로 재현됨). rclpy import 전에 환경을 잡아야 하므로
conftest 최상단에서 설정한다 — CLI 서브프로세스도 이 환경을 상속받는다.
"""

import os

os.environ['ROS_DOMAIN_ID'] = str(os.getpid() % 90 + 10)   # 10~99
os.environ['ROS_LOCALHOST_ONLY'] = '1'
