# CMakeLists.txt

cmake_minimum_required(VERSION 3.5)
project(user_gui)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 서비스 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendOrder.srv"
  "srv/CallStaff.srv"
)

# 파이썬 모듈 설치
ament_python_install_package(${PROJECT_NAME})

# 스크립트 설치
install(PROGRAMS
  user_gui/user_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# 종속성 연결
ament_package()




find_package(robot_services REQUIRED)

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
