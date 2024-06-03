## Robot Package for SLAM, Navigation and active SLAM using ROS2
## Setting up
Make a workspace folder (e.g., ros2_ws) and 'src' folder in your workspace. Clone this repository in the src folder. Go back to your workspace folder and build the package vander_one.
```bash
colcon build --packages-select vander_one --symlink-install
```

## Dependencies
Before running the launch files of this package, you have to make sure that the following packages are installed:
```bash
sudo apt install ros-humble-slam-toolbox
```
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

## Active SLAM Packages
For the implementation of active SLAM, I was able to find two packages that utilize navigation2 package.
Make sure to clone these in your 'src' folder:

[Package 1 by 'gjclifford'](https://github.com/gjcliff/SLAM-Frontier-Exploration)
[Package 2 by 'SeanReg'](https://github.com/SeanReg/nav2_wavefront_frontier_exploration)

While building package 2, the builder will throw an error that an email is missing. Just navigate to the package folder and add your own email where required (I think it is in 'package.xml' file. Ignore the other warnings in the build.

## Launching SLAM and active SLAM
Before launching SLAM, make sure you have built the packages and sourced your workspace. (Make sure you are in your 'ros2_ws' directory)
```bash
colcon build --symlink-install
```
```bash
source install/setup.bash
```

Finally, launch the SLAM simulation:
```bash
ros2 launch vander_one active_slam.launch.xml world:=./src/vander_one/worlds/arena.world
```
This will launch the gazebo simulator alongwith the ros2 controllers. At this point, keep your terminal open to check that the controllers have been properly configured and activated. Check for these in the terminal:
```bash
[spawner-6] [INFO] [1717237331.836888423] [spawner_diff_cont]: Loaded diff_cont
[gzserver-2] [INFO] [1717237331.838521565] [controller_manager]: Configuring controller 'diff_cont'
[spawner-6] [INFO] [1717237331.940031661] [spawner_diff_cont]: Configured and activated diff_cont
[gzserver-2] [INFO] [1717237331.941224009] [controller_manager]: Loading controller 'joint_broad'
[spawner-5] [INFO] [1717237331.983970133] [spawner_joint_broad]: Loaded joint_broad
[gzserver-2] [INFO] [1717237331.986144045] [controller_manager]: Configuring controller 'joint_broad'
[gzserver-2] [INFO] [1717237331.986339917] [joint_broad]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[spawner-5] [INFO] [1717237332.087690401] [spawner_joint_broad]: Configured and activated joint_broad
```

In another tab run rviz:
```bash
rviz2
```
Make sure you have these 3 topics selected:

![image](https://github.com/aeroSapphire/vander_one/assets/128461916/18c1045c-b858-446c-967e-b85d8da8db99)

The next step is running active SLAM. For that, we use the 'explore' node provided by the active SLAM packages I mentioned. The implementation is a bit different in both cases.

## Package 1
For package 1, make sure you have the other repository that's mentioned in that package's readme file also cloned to your src folder.
After you have launched SLAM, open another terminal and run:
```bash
ros2 run nubot_nav explore
```
It should start the autonomous SLAM but I have not been able to configure it properly. In the terminal where you launched SLAM, you will be able to see that its sending path to controllers but the robot does not move.

```bash
controller_server-8] [INFO] [1717236606.065079829] [controller_server]: Passing new path to controller.
[controller_server-8] [INFO] [1717236607.115161419] [controller_server]: Passing new path to controller.
[controller_server-8] [INFO] [1717236608.165135939] [controller_server]: Passing new path to controller.
[controller_server-8] [ERROR] [1717236608.865340620] [controller_server]: Failed to make progress
[controller_server-8] [WARN] [1717236608.865540143] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-8] [INFO] [1717236608.884334911] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap
[controller_server-8] [INFO] [1717236608.886765776] [controller_server]: Received a goal, begin computing control effort.
[controller_server-8] [WARN] [1717236609.024945240] [controller_server]: Control loop missed its desired rate of 20.0000Hz
```

Keep in mind that the package utilizes 'nav2_params.yaml' file that is in the config directory of this package. You might have to tune that to work this out. I am not sure.
## Package 2
For package 2, open a separate terminal and run:
```bash
ros2 run nav2_wfd explore
```

It should output this:
```bash
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL is deprecated. Use DurabilityPolicy.TRANSIENT_LOCAL instead.
  warnings.warn(
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE is deprecated. Use ReliabilityPolicy.RELIABLE instead.
  warnings.warn(
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
  warnings.warn(
[INFO] [1717237373.841682014] [nav2_waypoint_tester]: Running Waypoint Test
[INFO] [1717237373.842062909] [nav2_waypoint_tester]: Setting initial pose
[INFO] [1717237378.848367604] [nav2_waypoint_tester]: Waiting for amcl_pose to be received
[INFO] [1717237378.850887997] [nav2_waypoint_tester]: Setting initial pose
[INFO] [1717237383.856053221] [nav2_waypoint_tester]: Waiting for amcl_pose to be received
[INFO] [1717237383.858460409] [nav2_waypoint_tester]: Setting initial pose
[INFO] [1717237388.864018763] [nav2_waypoint_tester]: Waiting for amcl_pose to be received
[INFO] [1717237388.903447698] [nav2_waypoint_tester]: World points [(-0.14162650153819784, -0.0037803702251938205)]
[INFO] [1717237388.903765051] [nav2_waypoint_tester]: Sending goal request...
```

And again, the robot does not start moving.
