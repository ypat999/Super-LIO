cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fPIC")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(basic REQUIRED)

find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  message(STATUS "livox_ros_driver2 found, enabling Livox support")
  set(LIVOX_SUPPORT ON)
else()
  message(STATUS "livox_ros_driver2 not found, disabling Livox support")
  set(LIVOX_SUPPORT OFF)
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudPose.msg
  msg/CloudPose2.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

include_directories(
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  basic
  rosidl_default_runtime
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

if(LIVOX_SUPPORT)
  ament_target_dependencies(lio PUBLIC livox_ros_driver2)
  target_compile_definitions(lio PUBLIC LIVOX_SUPPORT=1)
endif()

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  pthread
  ${cpp_typesupport_target}
)

# ===== 下游（solution_a/b）库级引用 =====
# BUILD_INTERFACE: 本包内编译时用源码树；INSTALL_INTERFACE: 安装后头文件在
# include/<pkg>/ 下（见文件末尾 install(DIRECTORY include/) ），因此下游
# #include "lio/super_lio.h" 与 #include "super_lio/msg/cloud_pose.hpp" 都可见。
target_include_directories(lio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# OctVoxMap.hpp 是 header-only 且用到 ROOT 宏：下游包含该头时必须一并拿到定义，
# 否则模板实例化阶段直接编译失败。只做 INTERFACE 传递，不改变本包内已有行为。
target_compile_definitions(lio INTERFACE ROOT="${CMAKE_CURRENT_SOURCE_DIR}/")

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  ${cpp_typesupport_target}
)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  ${cpp_typesupport_target}
)

add_executable(dynamic_remove_node src/lio/dynamic_remove.cpp)
target_link_libraries(dynamic_remove_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  pthread
)

install(TARGETS
  lio
  EXPORT lioTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  super_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  dynamic_remove_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ===== 导出给下游：find_package(super_lio) + target_link_libraries(xxx lio) =====
ament_export_targets(lioTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  basic
  rosidl_default_runtime
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  Eigen3
  PCL
  Glog
)

# livox_ros_driver2 是 lio 的 PUBLIC 依赖（LIVOX_SUPPORT 开启时）：下游通过
# find_package(super_lio) 链接 super_lio::lio 时，必须一并被 find_dependency 出来，
# 否则 livox_ros_driver2::* 这些 IMPORTED target 不存在，下游 CMake 直接报错。
if(LIVOX_SUPPORT)
  ament_export_dependencies(livox_ros_driver2)
endif()

ament_package()
