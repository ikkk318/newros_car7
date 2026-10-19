cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯配置包：只安装 launch / params / maps / rviz，没有可执行文件需要编译。
install(DIRECTORY launch params maps rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
