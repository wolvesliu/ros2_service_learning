cmake_minimum_required(VERSION 3.8)
project(example_service_rclcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)#消息接口到包
find_package(example_interfaces REQUIRED)# 这里我们一次性把服务端和客户端对example_interfaces的依赖都加上

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

#新增：生成可执行文件node_01 节点到名称
add_executable(service_server_01 src/service_server_01.cpp)
add_executable(service_client_01 src/service_client_01.cpp)

#新增：添加依赖
ament_target_dependencies(service_server_01 rclcpp example_interfaces) 
ament_target_dependencies(service_client_01 rclcpp example_interfaces)

#新增：让编译器编译node_01这个文件，并输出到lib文件夹下
install(TARGETS service_server_01
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS service_client_01
  DESTINATION lib/${PROJECT_NAME}
)