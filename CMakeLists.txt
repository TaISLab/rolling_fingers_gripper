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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_ros2 REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# find installed library provided by dynamixel_ros2 package
# prefer exported targets / variables from find_package, fallback to find_library
# Try imported target first, then variables, then explicit search
set(DYNAMIXEL_ROS2_LIB "")
set(DYNAMIXEL_ROS2_TARGET "")

# If package provided an imported target
if(TARGET dynamixel_ros2_lib)
  set(DYNAMIXEL_ROS2_TARGET dynamixel_ros2_lib)
elseif(TARGET dynamixel_ros2::dynamixel_ros2_lib)
  set(DYNAMIXEL_ROS2_TARGET dynamixel_ros2::dynamixel_ros2_lib)
endif()

# If package provided library variables (from older CMake exports)
if(NOT DYNAMIXEL_ROS2_TARGET AND DEFINED dynamixel_ros2_LIBRARIES)
  set(DYNAMIXEL_ROS2_LIB ${dynamixel_ros2_LIBRARIES})
endif()

# last-resort: search common locations (kept for compatibility)
if(NOT DYNAMIXEL_ROS2_TARGET AND NOT DYNAMIXEL_ROS2_LIB)
  find_library(DYNAMIXEL_ROS2_LIB
    NAMES dynamixel_ros2_lib libdynamixel_ros2_lib dynamixel_ros2 libdynamixel_ros2
    PATHS
      ${CMAKE_PREFIX_PATH}
      ${CMAKE_PREFIX_PATH}/lib
      ${CMAKE_PREFIX_PATH}/lib/dynamixel_ros2
      ${CMAKE_PREFIX_PATH}/dynamixel_ros2/lib
      ${CMAKE_CURRENT_SOURCE_DIR}/../install/dynamixel_ros2/lib
      ${CMAKE_CURRENT_SOURCE_DIR}/../../install/dynamixel_ros2/lib
      /opt/ros/${ROS_DISTRO}/lib
      /usr/lib
    NO_DEFAULT_PATH
  )
  # Try explicit files in common install layouts if find_library failed
  if(NOT DYNAMIXEL_ROS2_LIB)
    foreach(prefix ${CMAKE_PREFIX_PATH} "/home/juanma/ros/rolling_fingers_ws/install/dynamixel_ros2" "/home/juanma/ros/rolling_fingers_ws/install")
      if(EXISTS "${prefix}/lib/dynamixel_ros2/libdynamixel_ros2_lib.so")
        set(DYNAMIXEL_ROS2_LIB "${prefix}/lib/dynamixel_ros2/libdynamixel_ros2_lib.so")
        break()
      elseif(EXISTS "${prefix}/lib/libdynamixel_ros2_lib.so")
        set(DYNAMIXEL_ROS2_LIB "${prefix}/lib/libdynamixel_ros2_lib.so")
        break()
      elseif(EXISTS "${prefix}/lib/dynamixel_ros2/libdynamixel_ros2_lib.lib")
        set(DYNAMIXEL_ROS2_LIB "${prefix}/lib/dynamixel_ros2/libdynamixel_ros2_lib.lib")
        break()
      endif()
    endforeach()
  endif()
endif()

# Don't fail the whole build; prefer warning and continue so other packages can still build.
if(DYNAMIXEL_ROS2_TARGET OR DYNAMIXEL_ROS2_LIB)
  message(STATUS "Found dynamixel_ros2: target='${DYNAMIXEL_ROS2_TARGET}' lib='${DYNAMIXEL_ROS2_LIB}'")
else()
  message(WARNING "Could not find dynamixel_ros2 library/target.  Make sure dynamixel_ros2 was built and 'source install/setup.bash' was run. Nodes will still configure but may fail to link/run if the library is unavailable.")
endif()

# executables
add_executable(rolling_3_fingers src/rolling_3_fingers.cpp)
ament_target_dependencies(rolling_3_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(rolling_3_fingers PRIVATE include)
if(DYNAMIXEL_ROS2_TARGET)
  target_link_libraries(rolling_3_fingers ${DYNAMIXEL_ROS2_TARGET})
elseif(DYNAMIXEL_ROS2_LIB)
  target_link_libraries(rolling_3_fingers ${DYNAMIXEL_ROS2_LIB})
endif()
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(rolling_3_fingers dynamixel_sdk::dynamixel_sdk)
endif()

add_executable(fsm_3_fingers src/fsm_3_fingers.cpp)
ament_target_dependencies(fsm_3_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(fsm_3_fingers PRIVATE include)
if(DYNAMIXEL_ROS2_TARGET)
  target_link_libraries(fsm_3_fingers ${DYNAMIXEL_ROS2_TARGET})
elseif(DYNAMIXEL_ROS2_LIB)
  target_link_libraries(fsm_3_fingers ${DYNAMIXEL_ROS2_LIB})
endif()
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(fsm_3_fingers dynamixel_sdk::dynamixel_sdk)
endif()

add_executable(rolling_4_fingers src/rolling_4_fingers.cpp)
ament_target_dependencies(rolling_4_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(rolling_4_fingers PRIVATE include)
if(DYNAMIXEL_ROS2_TARGET)
  target_link_libraries(rolling_4_fingers ${DYNAMIXEL_ROS2_TARGET})
elseif(DYNAMIXEL_ROS2_LIB)
  target_link_libraries(rolling_4_fingers ${DYNAMIXEL_ROS2_LIB})
endif()
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(rolling_4_fingers dynamixel_sdk::dynamixel_sdk)
endif()

add_executable(fsm_4_fingers src/fsm_4_fingers.cpp)
ament_target_dependencies(fsm_4_fingers rclcpp std_msgs dynamixel_ros2)
target_include_directories(fsm_4_fingers PRIVATE include)
if(DYNAMIXEL_ROS2_TARGET)
  target_link_libraries(fsm_4_fingers ${DYNAMIXEL_ROS2_TARGET})
elseif(DYNAMIXEL_ROS2_LIB)
  target_link_libraries(fsm_4_fingers ${DYNAMIXEL_ROS2_LIB})
endif()
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(fsm_4_fingers dynamixel_sdk::dynamixel_sdk)
endif()

# installation
install(TARGETS
  rolling_3_fingers
  fsm_3_fingers
  rolling_4_fingers
  fsm_4_fingers
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
endif()

# optionally install launch files if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
