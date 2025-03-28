cmake_minimum_required(VERSION 3.8)
project(rm_detector)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rm_interfaces REQUIRED)

# 添加可执行文件
add_executable(rm_detector_node 
  src/armor_detector_opencv_node.cpp
  src/armor_detector_opencv.cpp
)

# 指定头文件路径（推荐使用 target_include_directories）
target_include_directories(rm_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖项（使用 ament_target_dependencies 自动处理）
ament_target_dependencies(rm_detector_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  rm_interfaces
)

# 安装目标
install(TARGETS rm_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 测试相关配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查（如果需要正式发布，应添加版权信息）
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查（如果在 git 仓库中，应启用）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 必须放在最后
ament_package()