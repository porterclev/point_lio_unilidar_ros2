cmake_minimum_required(VERSION 3.8)
project(lidar_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Set policy CMP0074
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  ${sensor_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${visualization_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  include
)

include_directories(
  ${PCL_INCLUDE_DIRS} 
  include
  unitree_lidar_sdk/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)

add_executable(laser_mapping 
src/laserMapping.cpp 
include/ikd-Tree/ikd_Tree.cpp 
  src/parameters.cpp 
  src/preprocess.cpp 
  src/Estimator.cpp
)

add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

ament_target_dependencies(laser_mapping
  rclcpp
  tf2_eigen
  visualization_msgs  
  nav_msgs
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  rcl_interfaces
  rcl
  rcpputils  
)
install(TARGETS
    laser_mapping
  DESTINATION lib/${PROJECT_NAME})
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
