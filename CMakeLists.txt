cmake_minimum_required(VERSION 3.8)
project(rolling_fingers_gripper)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_ros2 REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# locate installed library file (built in workspace install/ or system prefixes)
find_library(DYNAMIXEL_ROS2_LIB
  NAMES dynamixel_ros2_lib libdynamixel_ros2_lib
  PATHS ${CMAKE_PREFIX_PATH}
  NO_DEFAULT_PATH
)
if(NOT DYNAMIXEL_ROS2_LIB)
  find_library(DYNAMIXEL_ROS2_LIB NAMES dynamixel_ros2_lib libdynamixel_ros2_lib)
endif()
message(STATUS "Found dynamixel_ros2 lib: ${DYNAMIXEL_ROS2_LIB}")

# executables (one after another)
add_executable(rolling_3_fingers src/rolling_3_fingers.cpp)
ament_target_dependencies(rolling_3_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(rolling_3_fingers PRIVATE include)
target_link_libraries(rolling_3_fingers ${DYNAMIXEL_ROS2_LIB} dynamixel_sdk::dynamixel_sdk)

add_executable(fsm_3_fingers src/fsm_3_fingers.cpp)
ament_target_dependencies(fsm_3_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(fsm_3_fingers PRIVATE include)
target_link_libraries(fsm_3_fingers ${DYNAMIXEL_ROS2_LIB} dynamixel_sdk::dynamixel_sdk)

add_executable(rolling_4_fingers src/rolling_4_fingers.cpp)
ament_target_dependencies(rolling_4_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(rolling_4_fingers PRIVATE include)
target_link_libraries(rolling_4_fingers ${DYNAMIXEL_ROS2_LIB} dynamixel_sdk::dynamixel_sdk)

add_executable(fsm_4_fingers src/fsm_4_fingers.cpp)
ament_target_dependencies(fsm_4_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(fsm_4_fingers PRIVATE include)
target_link_libraries(fsm_4_fingers ${DYNAMIXEL_ROS2_LIB} dynamixel_sdk::dynamixel_sdk)

# installation
install(TARGETS
  rolling_3_fingers
  fsm_3_fingers
  rolling_4_fingers
  fsm_4_fingers
  DESTINATION lib/${PROJECT_NAME}
)

# install include dir only if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
endif()

ament_package()
