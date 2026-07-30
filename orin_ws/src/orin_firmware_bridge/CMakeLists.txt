cmake_minimum_required(VERSION 3.8)
project(orin_firmware_bridge)

# C++ 표준 설정 (ROS2 Humble은 C++17 이상 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 확인
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mgs01_base_msgs REQUIRED)
find_package(mgs_tp_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)  # run_profile action 서버 (testbed_spec §3.2)

# LibSerial을 위한 pkg-config 설정
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 프로파일 YAML 파싱 (testbed_spec.md §4) — libyaml-cpp-dev
find_package(yaml-cpp REQUIRED)

# 2. 라이브러리 정의 — 계층별 2분할 (redesign/02 §4.1, 6단계)
#
#   rd_core_lib : L0~L2. **rclcpp 를 링크하지 않는다.**
#   rd_ros_lib  : L3.    ROS 인터페이스 (여기만 rclcpp 를 안다)
#
# 이 분리가 하는 일: 하위 계층에서 실수로 rclcpp 를 쓰면 **링크 에러가 난다.**
# 문서 규약은 잊히지만 빌드 실패는 잊히지 않는다 — 그것이 이 단계의 전부다.

add_library(rd_core_lib SHARED
  src/core/rd_uart.cpp            # L0 전송
  src/core/rd_comm.cpp
  src/core/rd_map.cpp
  src/core/rd_clock_sync.cpp
  src/rd_logger.cpp          # 로거·시계 기본 구현 (A5)
  src/sched/rd_schedule.cpp        # L1 스케줄
  src/policy/rd_control.cpp         # L2 제어 FSM (구 rd_testbed — 01 §8.1 개명)
  src/policy/rd_status.cpp          # GET_STATUS 정형 JSON 조립 (04 §4 C3) — rclcpp 불필요
  src/policy/rd_profile.cpp
  src/policy/rd_profile_player.cpp
  src/policy/rd_oos.cpp
  src/policy/rd_command.cpp
  src/policy/rd_sequence.cpp        # L2 jeongae 전개 FSM (02 A3) — rd_command 에서 분리
)
target_include_directories(rd_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)
# libserial 과 yaml-cpp 만. **rclcpp 는 의도적으로 없다.**
target_link_libraries(rd_core_lib ${LIBSERIAL_LIBRARIES} yaml-cpp)

add_library(rd_ros_lib SHARED
  src/ros/rd_node.cpp          # L3 조립 (rd_node 역할)
  src/ros/rd_telemetry.cpp       # L3 발행
  src/ros/rd_carrier_api.cpp     # L3 project 계약
  src/ros/rd_control_api.cpp     # L3 control 계약
)
target_include_directories(rd_ros_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rd_ros_lib rd_core_lib)
ament_target_dependencies(rd_ros_lib
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs std_srvs mgs01_base_msgs mgs_tp_msgs)

# 기존 이름 유지 — 하위 타깃/테스트가 rd_bridge_lib 를 참조한다.
# (두 라이브러리를 함께 끌어오는 인터페이스 타깃)
add_library(rd_bridge_lib INTERFACE)
target_link_libraries(rd_bridge_lib INTERFACE rd_ros_lib rd_core_lib)

# 3. 테스트용 실행 파일 등록
add_executable(comm_test_node src/main.cpp)
target_link_libraries(comm_test_node
  rd_bridge_lib
  pthread
)

# command_cli 삭제 (06 Q4/C4) — control_cli 의 `command` 하위명령으로 흡수했다.
#   같은 브리지를 조작하는 도구가 둘이면 조작자가 어느 쪽을 쓰느냐에 따라 가능한 일이 달라진다.

# 4. 설치 설정 (ros2 run 및 타 패키지 참조용)
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS rd_core_lib rd_ros_lib rd_bridge_lib comm_test_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 5. ROS 2 환경 수출 (중요: export_targets 이름은 위 install과 일치해야 함)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mgs01_base_msgs mgs_tp_msgs)

# 5. 테스트 (testbed_spec 검증 하네스 — ECU 하드웨어 없이 bridge 로직을 돌린다)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 노드 직접 구성 + tick 모사로 도는 하네스들
  foreach(_t
      test_control_fsm          # C-2 FSM
      test_profile_parser       # C-4a 프로파일 파서
      test_run_profile_action   # C-4b run_profile action
      test_config_service       # C-5 config service
      test_auto_mode            # C-8 auto_mode 연동
      test_interface_inventory) # 06 §2.4 인터페이스 인벤토리 (5·7단계 관문)
    ament_add_gtest(${_t} test/integration/${_t}.cpp TIMEOUT 180)
    if(TARGET ${_t})
      target_include_directories(${_t} PRIVATE test/integration)
      target_link_libraries(${_t} rd_bridge_lib)
      ament_target_dependencies(${_t} rclcpp rclcpp_action mgs01_base_msgs mgs_tp_msgs)
    endif()
  endforeach()

  # IDLE 안전값 (01 §6.1.2) — L2 만 쓰므로 rclcpp·노드 불필요 (A5 의 성과).
  ament_add_gtest(test_idle_safe_value test/unit/test_idle_safe_value.cpp TIMEOUT 60)
  if(TARGET test_idle_safe_value)
    target_link_libraries(test_idle_safe_value rd_core_lib)
  endif()

  # GET_STATUS 정형 JSON (04 §4 C3) — 스키마가 계약이라 키·타입을 고정한다.
  ament_add_gtest(test_status_json test/unit/test_status_json.cpp TIMEOUT 60)
  if(TARGET test_status_json)
    target_link_libraries(test_status_json rd_core_lib)
  endif()

  # 슬롯 테이블 = **스케줄 골든** (04 §2.2, 00 C1).
  #   골든 바이트가 못 보는 것을 본다: 어느 tick 에 어느 태스크가 나가는지.
  ament_add_gtest(test_slot_table test/unit/test_slot_table.cpp TIMEOUT 60)
  if(TARGET test_slot_table)
    target_include_directories(test_slot_table PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # B6 의미 단위 명령 카탈로그 + 게이트 (00 B6, 03 §7).
  #   허용/거부 조합이 **순수 함수 하나**에 모여 있다는 것을 고정한다 — 호출부마다
  #   조건을 다시 적으면 그 순간 갈라진다.
  ament_add_gtest(test_b6_catalog test/unit/test_b6_catalog.cpp TIMEOUT 60)
  if(TARGET test_b6_catalog)
    target_include_directories(test_b6_catalog PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # jeongae 전개 FSM (02 A3) — 가짜 슬롯 호스트만으로 돈다. 분리의 값어치가 여기 있다.
  ament_add_gtest(test_sequence test/unit/test_sequence.cpp TIMEOUT 60)
  if(TARGET test_sequence)
    target_link_libraries(test_sequence rd_core_lib)
  endif()

  # bridge_mode 열거형 + manual (00 Q1·R1·R2 / 01 §4) — 순수 헤더.
  ament_add_gtest(test_bridge_mode test/unit/test_bridge_mode.cpp TIMEOUT 60)
  if(TARGET test_bridge_mode)
    target_include_directories(test_bridge_mode PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # C2 — once 만료가 시간이 아니라 시도 횟수인지 (00 C2).
  ament_add_gtest(test_c2_once_attempts test/unit/test_c2_once_attempts.cpp TIMEOUT 60)
  if(TARGET test_c2_once_attempts)
    target_link_libraries(test_c2_once_attempts rd_core_lib)
  endif()

  # E3 — noise 의 slew 면제 + 4σ 진폭 가드 (05 §3.4).
  ament_add_gtest(test_e3_noise_slew test/unit/test_e3_noise_slew.cpp TIMEOUT 60)
  if(TARGET test_e3_noise_slew)
    target_link_libraries(test_e3_noise_slew rd_core_lib)
  endif()

  # 읽기 프리셋 테이블 (04 §2.3) — Covers() 의미와 기본 프리셋 불변식.
  ament_add_gtest(test_read_preset test/unit/test_read_preset.cpp TIMEOUT 60)
  if(TARGET test_read_preset)
    target_link_libraries(test_read_preset rd_core_lib)
  endif()

  # DIRECT 의 모터별 단위 라우팅 (01 §6.1.2 표 마지막 행) — 각도가 암페어로 새는 것을 막는다.
  ament_add_gtest(test_direct_units test/unit/test_direct_units.cpp TIMEOUT 60)
  if(TARGET test_direct_units)
    target_link_libraries(test_direct_units rd_core_lib)
  endif()

  # STREAM + arm 게이트 (01 §6.1.3) — 가짜 시계로 스테일을 결정론적으로 만든다.
  ament_add_gtest(test_stream_arm test/unit/test_stream_arm.cpp TIMEOUT 60)
  if(TARGET test_stream_arm)
    target_link_libraries(test_stream_arm rd_core_lib)
  endif()

  # 공유 프로파일 코퍼스 (05 §3.5) — C++/Python 판정 일치 고정
  ament_add_gtest(test_profile_corpus test/unit/test_profile_corpus.cpp TIMEOUT 60)
  if(TARGET test_profile_corpus)
    target_link_libraries(test_profile_corpus rd_core_lib)
    target_compile_definitions(test_profile_corpus PRIVATE
      RD_CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/profiles")
  endif()

  # 큐 포화 테스트 (06 §2.3) — 골든이 못 보는 발행 경로를 본다. 순수 헤더라 링크 불필요.
  ament_add_gtest(test_txn_queue test/unit/test_txn_queue.cpp TIMEOUT 60)
  if(TARGET test_txn_queue)
    target_include_directories(test_txn_queue PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # 카운터 생산자 테스트 (06 §9.5) — **소스를 읽는다.** 값을 보는 테스트로는 못 잡는다:
  #   생산자가 없는 카운터는 늘 0 이고, 0 은 "사고가 없었다" 와 구분이 안 된다.
  #   irregular_tick_cnt 가 실제로 그 상태로 방치돼 있었다.
  ament_add_gtest(test_counter_producers test/unit/test_counter_producers.cpp TIMEOUT 60)
  if(TARGET test_counter_producers)
    target_compile_definitions(test_counter_producers PRIVATE
      RD_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  # 죽은 코드 재발 방지 (08 §2.1 ①·§2.2 ⑤) — 위와 같은 이유로 **소스를 읽는다.**
  #   정의만 있고 아무도 안 부르는 함수, 사용처가 0 인 매크로, 선언만 남은 멤버는
  #   컴파일러도 링커도 오류로 보지 않는다. 이름으로 막는 수밖에 없다.
  ament_add_gtest(test_no_dead_code test/unit/test_no_dead_code.cpp TIMEOUT 60)
  if(TARGET test_no_dead_code)
    target_compile_definitions(test_no_dead_code PRIVATE
      RD_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  # 읽기 전용 control 의 INIT 처리 (2026-07-30 실기에서 발견) — **소스를 읽는다.**
  #   이 분기는 시리얼 포트가 열려야 도달하므로 노드만 띄워서는 재현되지 않는다.
  #   견인 실험(`./run.sh traction`)이 이 구성이고, INIT 에 머물면 조작 입구가 통째로 막힌다.
  ament_add_gtest(test_init_read_only test/unit/test_init_read_only.cpp TIMEOUT 60)
  if(TARGET test_init_read_only)
    target_compile_definitions(test_init_read_only PRIVATE
      RD_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  # 계층 경계 (02 §4, A3) — 디렉터리를 나눈 값어치는 경계가 지켜질 때만 생긴다.
  #   위 2분할이 링크로 막는 것 **밖**을 본다: 헤더만 include 한 위반과 역방향 의존.
  #   02 §4.1 이 "CI 보조 검사 (선택)" 로 적어 둔 grep 을 테스트로 옮긴 것이다.
  ament_add_gtest(test_layer_boundary test/unit/test_layer_boundary.cpp TIMEOUT 60)
  if(TARGET test_layer_boundary)
    target_compile_definitions(test_layer_boundary PRIVATE
      RD_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  # 골든 바이트 테스트 — RS485 와이어 계약 고정 (redesign/06 §2.2, 0단계).
  #   재설계가 RdNode 를 4분할하면 기존 하네스는 검증 대상을 잃는다. 분할 대상이 아닌
  #   L1(RdMap) 의 출력 바이트열을 고정해 "동작 불변" 을 바이트 단위로 증명한다.
  #   골든 파일은 **소스 트리**에 둔다 (빌드 디렉터리는 지워지므로).
  ament_add_gtest(test_golden_wire test/integration/test_golden_wire.cpp TIMEOUT 60)
  if(TARGET test_golden_wire)
    target_link_libraries(test_golden_wire rd_bridge_lib)
    ament_target_dependencies(test_golden_wire rclcpp mgs01_base_msgs mgs_tp_msgs)
    target_compile_definitions(test_golden_wire PRIVATE
      RD_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/golden/ecu_wire.txt")
  endif()

  # STM 펌웨어 헤더와 Orin 미러의 바이트 단위 대조 (redesign/01 §9.1).
  #   두 헤더는 손으로 동기화되고, 어긋나도 빌드·테스트가 통과하면서 브리지가 엉뚱한 주소를
  #   읽는다. rd_register_ecu.hpp 의 static_assert 는 **크기만** 보므로 필드 순서 변경을 못 잡는다.
  #   stm_ws 가 함께 있을 때만 빌드한다 — Orin 에 배포된 워크스페이스에는 없으므로 조용히 빠진다.
  set(_stm_inc "${CMAKE_CURRENT_SOURCE_DIR}/../../../stm_ws/ECU_V3/Core/Inc")
  if(EXISTS "${_stm_inc}/rd_register_ecu.h")
    # 버퍼 상수는 HAL 을 끌어오는 헤더(rd_uart.h)에 있어 호스트에서 include 할 수 없다.
    # 값만 정규식으로 뽑아 -D 로 넘긴다 — 헤더를 고치면 이 값도 따라 바뀐다.
    function(_stm_define out_var file macro)
      file(STRINGS "${_stm_inc}/${file}" _line REGEX "^[ \t]*#define[ \t]+${macro}[ \t]+[0-9]+")
      string(REGEX MATCH "[0-9]+" _val "${_line}")
      set(${out_var} "${_val}" PARENT_SCOPE)
    endfunction()
    _stm_define(_stm_rx  rd_uart.h      RX_BUFFER_SIZE)
    _stm_define(_stm_tx  rd_uart.h      TX_BUFFER_SIZE)
    _stm_define(_stm_pkt rd_comm_ecu.h  PACKET_DATA_BUF_SIZE)

    ament_add_gtest(test_stm_mirror test/unit/test_stm_mirror.cpp TIMEOUT 60)
    if(TARGET test_stm_mirror)
      target_include_directories(test_stm_mirror PRIVATE
        "${_stm_inc}" "${CMAKE_CURRENT_SOURCE_DIR}/include")   # 순수 헤더 테스트 — 라이브러리 링크 불필요
      target_compile_definitions(test_stm_mirror PRIVATE
        STM_RX_BUFFER_SIZE=${_stm_rx}
        STM_TX_BUFFER_SIZE=${_stm_tx}
        STM_PACKET_DATA_BUF_SIZE=${_stm_pkt})
    endif()
  else()
    message(STATUS "stm_ws 없음 — test_stm_mirror 생략 (${_stm_inc})")
  endif()

  # C-3 은 계약이 '프로세스 종료 코드' 라 실제 노드 바이너리를 띄워 확인한다
  ament_add_gtest(test_active_motors test/integration/test_active_motors.cpp TIMEOUT 180)
  if(TARGET test_active_motors)
    target_include_directories(test_active_motors PRIVATE test/integration)
    target_compile_definitions(test_active_motors PRIVATE
      COMM_TEST_NODE_PATH="$<TARGET_FILE:comm_test_node>")
    ament_target_dependencies(test_active_motors rclcpp)
    add_dependencies(test_active_motors comm_test_node)
  endif()
endif()

if(FALSE)  # 기존 ament_lint 블록 (아래 유지)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()