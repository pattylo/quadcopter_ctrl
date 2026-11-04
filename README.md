```
sudo apt install ros-${ROS_DISTRO}-sophus \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-mavros-extras \
  ros-${ROS_DISTRO}-mavros-msgs
```

Build and launch the controller with MAVROS already running:

```bash
cd ros2_ws
colcon build --packages-select quadcopter_ctrl --symlink-install
source install/setup.bash
ros2 launch quadcopter_ctrl ctrl.launch.py
```

Edit `launch/trajectory.yaml`: select `trajectory.type` first, then edit the matching
`hover`, `circle`, `lemniscate`, `square`, or `step` block.

Set `mavros_namespace` in that YAML to match MAVROS, for example `/lala/mavros`
when the pose topic is `/lala/mavros/local_position/pose`. Launch with the same
command above; no extra launch arguments are needed.
