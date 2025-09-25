# rolling_fingers_gripper

Control nodes and examples for the Rolling Fingers gripper (3- and 4-finger variants). The nodes use the local wrapper library `dynamixel_ros2` which in turn uses the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

This README explains required dependencies, how to build the package in a workspace, and how to run the example nodes.

# Dependencies

This repo uses the [dynamixel_ros2](https://github.com/jmgandarias/dynamixel_ros2).

To be able to use, you'll need to do the following:

- Install the [dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

```bash
sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk
```

- Create a ros2 workspace where you will include this repository
- In yout `src` folder, clone the [dynamixel_ros2](https://github.com/jmgandarias/dynamixel_ros2):

```bash
git clone git@github.com:jmgandarias/dynamixel_ros2.git
```

- Then, clone the rolling_fingers_gripper repository

```bash
git clone git@github.com:TaISLab/rolling_fingers_gripper.git
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Dynamixel SDK (ROS package)
  ```bash
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-dynamixel-sdk
  ```

## Quick install (recommended)

1. Create or go to a workspace:
   ```bash
   mkdir -p ~/ros/rolling_fingers_ws/src
   cd ~/ros/rolling_fingers_ws/src
   ```

2. Clone required repositories into `src/`:
   ```bash
   # dynamixel wrapper library used by these nodes
   git clone git@github.com:jmgandarias/dynamixel_ros2.git

   # this repository (rolling_fingers_gripper)
   git clone git@github.com:TaISLab/rolling_fingers_gripper.git
   ```

3. Build the workspace and source the overlay:
   ```bash
   cd ~/ros/rolling_fingers_ws
   colcon build
   source install/setup.bash
   ```

## Running the example nodes

After sourcing the workspace overlay, run one of the example executables:

- 3-finger node:
  ```bash
  ros2 run rolling_fingers_gripper rolling_3_fingers
  ```

- 4-finger node:
  ```bash
  ros2 run rolling_fingers_gripper rolling_4_fingers
  ```

If you add launch files, use `ros2 launch <package> <launchfile>`.

## Configuration

- Serial device: ensure the node can access your serial device (e.g. `/dev/ttyUSB0`). Add your user to the `dialout` group if needed:
  ```bash
  sudo usermod -a -G dialout $USER
  # then re-login or open a new shell
  ```

- Motor IDs, baud rate and port are configured in the node source files or via parameters if exposed — check `src/rolling_*_fingers.cpp` for current defaults.

## Using the dynamixel_ros2 library from another package

If you want to use the wrapper library directly from another ROS 2 package:

- package.xml
  ```xml
  <build_depend>dynamixel_ros2</build_depend>
  <exec_depend>dynamixel_ros2</exec_depend>
  <build_depend>dynamixel_sdk</build_depend>
  <exec_depend>dynamixel_sdk</exec_depend>
  ```

- CMakeLists.txt (minimal example)
  ```cmake
  find_package(dynamixel_ros2 REQUIRED)
  find_package(dynamixel_sdk REQUIRED)

  add_executable(my_node src/my_node.cpp)
  ament_target_dependencies(my_node rclcpp dynamixel_ros2 dynamixel_sdk)

  # link exported target if provided by the library
  if(TARGET dynamixel_ros2_lib)
    target_link_libraries(my_node dynamixel_ros2_lib)
  endif()
  ```

Build your package after sourcing the workspace install overlay that contains `dynamixel_ros2`.

## Troubleshooting

- Linker errors (undefined references):
  - Make sure you built `dynamixel_ros2` and sourced the workspace `install/setup.bash`.
  - Verify library presence: `ls -l install/dynamixel_ros2/lib` or `/opt/ros/$ROS_DISTRO/lib`.

- AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH warnings:
  - These indicate a stale overlay in your environment. Re-source the correct `install/setup.bash` or remove old overlay entries.

- Serial port permission issues:
  - Add user to `dialout` group or run node with appropriate permissions.

## Development notes

- Example nodes are minimal for demonstration. Add parameter support, safety checks and launch files for production use.
- If you change the exported CMake target name in `dynamixel_ros2`, update dependent packages accordingly.

## License

This repository uses the `dynamixel_ros2` library which is licensed under the GNU GPLv3 (see `src/dynamixel_ros2/LICENSE`). Ensure you follow the GPLv3 obligations when redistributing derived works.
