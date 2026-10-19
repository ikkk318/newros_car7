cmake_minimum_required(VERSION 3.8)
# 声明功能包名称：agv_interfaces（自定义消息 / 服务接口包）
project(agv_interfaces)

# 编译警告选项：对所有编译单元开启严格警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖：构建工具与 rosidl 接口代码生成器
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 接口生成：把 msg/ 与 srv/ 下的定义编译成 Python 可用的消息、服务类型
# 共 6 个：3 个 msg + 3 个 srv（原先是 12 个：点位/路线的 8 个同构 CRUD 服务
# 已合并为 1 个 AgvStore；新增 NamedPose 统一表示"带名字的位姿"）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NamedPose.msg"
  "msg/AgvCommand.msg"
  "msg/TaskState.msg"
  "srv/AgvStore.srv"
  "srv/AgvQuery.srv"
  "srv/CargoCmd.srv"
)

# 声明生成 ament 包（必须放在文件末尾）
ament_package()
