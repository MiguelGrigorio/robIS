# robis_description

ROS 2 Jazzy description package for the RobIS D2X platform.

## What is included

- Canonical `robis.urdf.xacro` based on the validated RobIS geometry
- Centralized robot parameters in `config/robot_parameters.yaml`
- Chassis mesh in `meshes/chassis.stl`
- `display.launch.py` for RViz visualization
- Minimal RViz configuration in `rviz/robis.rviz`

## Folder layout

```text
robis_description/
├── CMakeLists.txt
├── package.xml
├── config/
├── launch/
├── meshes/
├── rviz/
└── urdf/
```

## Suggested usage on the remote Jazzy container

Copy this package into the workspace source folder:

```bash
cd ~/robis_ws/src
cp -R /path/to/robis_description .
```

Install dependencies and build:

```bash
cd ~/robis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robis_description
source install/setup.bash
```

Launch the robot model in RViz:

```bash
ros2 launch robis_description display.launch.py
```

If you want the joint sliders:

```bash
ros2 launch robis_description display.launch.py use_joint_state_publisher_gui:=true
```

## Notes

- This package is intentionally focused on robot description first.
- The Gazebo tags are already embedded in the xacro so the next simulation package can reuse the same robot model.
- The parameter values currently follow `phis/robis.urdf.xacro`, which was defined as the most reliable source.
