# robis_gazebo

Gazebo Sim package for the RobIS D2X platform on ROS 2 Jazzy.

## What this package provides

- A simple indoor world inspired by the TurtleBot3 navigation demos
- Launch files to open Gazebo Sim and spawn the robot
- A bridge configuration for the main topics used in the project:
  - `/clock`
  - `/demo/cmd_vel`
  - `/demo/odom`
  - `/demo/scan`
  - `/demo/camera/image_raw`
  - `/demo/camera/camera_info`
  - `/demo/tf`

## Suggested usage

```bash
cd ~/robis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robis_description robis_gazebo
source install/setup.bash
ros2 launch robis_gazebo simulation.launch.py
```

## Expected first checks

After launching, validate:

```bash
ros2 topic list | grep -E '/demo/(odom|scan|camera|cmd_vel|tf)'
ros2 topic echo /demo/odom --once
ros2 topic echo /demo/scan --once
ros2 topic echo /demo/camera/camera_info --once
ros2 topic pub /demo/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.2}}' -r 10
```

## Notes

- This package assumes Gazebo Sim with `ros_gz_sim` on ROS 2 Jazzy.
- The robot description is generated from `robis_description/urdf/robis.urdf.xacro` with `sim_flavor:=gz`.
- The default world is `worlds/turtlebot3_like.world`, but you can still pass `world:=.../empty.world` if you want a simpler scene.
- The goal here is phase-0 validation: spawn, move, publish sensors and establish the baseline for Nav2 and SLAM.
