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
find_package(mgs01_base_msgs REQUIRED)

# LibSerial을 위한 pkg-config 설정
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 2. 라이브러리 정의 (공통 통신 엔진)
add_library(rd_bridge_lib SHARED
  src/rd_uart.cpp
  src/rd_comm.cpp
  src/rd_bridge.cpp
  src/rd_map.cpp
  src/rd_schedule.cpp
)

# 인클루드 경로 (내부/외부 모두 참조 가능하게)
target_include_directories(rd_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

# 링킹 설정
target_link_libraries(rd_bridge_lib 
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(rd_bridge_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mgs01_base_msgs
)

# 3. 테스트용 실행 파일 등록
add_executable(comm_test_node src/main.cpp)
target_link_libraries(comm_test_node
  rd_bridge_lib
  pthread
)

# 4. 설치 설정 (ros2 run 및 타 패키지 참조용)
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS rd_bridge_lib comm_test_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 5. ROS 2 환경 수출 (중요: export_targets 이름은 위 install과 일치해야 함)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs)

if(BUILD_TESTING)
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