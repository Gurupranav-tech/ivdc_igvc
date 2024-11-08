################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rplidar_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

if(APPLE)
  add_compile_definitions(_MACOS)
  FILE(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/macOS/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
else()
  FILE(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(numpy REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
################################################################################
# Build
################################################################################
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

#Python node dependencies
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src)

# RPLIDAR Node
add_executable(rplidar_node src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(rplidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

# RPLIDAR Composition
add_executable(rplidar_composition src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(rplidar_composition
  rclcpp
  std_srvs
  sensor_msgs
)

# RPLIDAR Client
add_executable(rplidar_client src/rplidar_client.cpp)
ament_target_dependencies(rplidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

# Lidar Map Node
add_executable(lidar_map_node src/lidar_map_node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(lidar_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/igvc_slam_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/cv_to_map2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/imu.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rplidar_node rplidar_composition rplidar_client lidar_map_node 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(rclpy)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(numpy)

ament_export_include_directories(include)
ament_package()
