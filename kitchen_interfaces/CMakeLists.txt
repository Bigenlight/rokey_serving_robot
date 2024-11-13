cmake_minimum_required(VERSION 3.5)
project(kitchen_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 및 서비스 파일 설정
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CallService.msg"
  "srv/OrderService.srv"
)

ament_package()
