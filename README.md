# rolling_fingers_gripper
This is the repo for controlling the rolling fingers gripper

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
