cmake_minimum_required(VERSION 3.8)
project(agv_description)

find_package(ament_cmake REQUIRED)

# 纯资源包（URDF / launch / rviz / world / config），没有可执行文件需要编译。
# urdf/ 目录含 base/actuator/sensor/plugins 子目录（模块化结构），整体安装即可。
install(DIRECTORY urdf launch rviz worlds config
        DESTINATION share/${PROJECT_NAME})

ament_package()
