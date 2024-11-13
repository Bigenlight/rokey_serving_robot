cmake_minimum_required(VERSION 3.5)
project(robot_services)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 서비스 파일 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendOrder.srv"
  "srv/CallStaff.srv"
  DEPENDENCIES std_msgs
)

# Python 패키지 설치
ament_python_install_package(${PROJECT_NAME})

# 스크립트 설치 부분 제거
# install(PROGRAMS
#   robot_services/kitchen_service.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)

# 패키지 선언
ament_package()
