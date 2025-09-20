cmake_minimum_required(VERSION 3.8)
project(rolling_fingers_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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

ament_package()


# 
add_executable(rolling_3_fingers src/rolling_3_fingers.cpp)
add_dependencies(rolling_3_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_3_fingers
  dynamixel_ros
  ${catkin_LIBRARIES}
)

add_executable(fsm_3_fingers src/fsm_3_fingers.cpp)
add_dependencies(fsm_3_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsm_3_fingers
  dynamixel_ros
  ${catkin_LIBRARIES}
)

add_executable(rolling_4_fingers src/rolling_4_fingers.cpp)
add_dependencies(rolling_4_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_4_fingers
  dynamixel_ros
  ${catkin_LIBRARIES}
)

add_executable(fsm_4_fingers src/fsm_4_fingers.cpp)
add_dependencies(fsm_4_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsm_4_fingers
  dynamixel_ros
  ${catkin_LIBRARIES}
)
