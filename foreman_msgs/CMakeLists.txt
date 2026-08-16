cmake_minimum_required(VERSION 3.8)
project(foreman_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/ComponentState.msg"
  "msg/ForemanErrorState.msg"
)

set(msg_files
  "msg/ComponentState.msg"
  "msg/ForemanErrorState.msg"
  "msg/ForemanStatus.msg"
)

set(srv_files
  "srv/SetGoal.srv"
)

set(action_files
  "action/SetGoal.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
