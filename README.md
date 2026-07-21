<div align="center">

# OOMWOO One

*Open-source robot vacuum you build yourself.*

ROS 2 Jazzy · Gazebo · URDF · Nav2 · SLAM · 2D LiDAR · robot description

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Status](https://img.shields.io/badge/status-sim%20ready-brightgreen)
[![Part of OOMWOO](https://img.shields.io/badge/part%20of-OOMWOO-5eead4)](https://github.com/makerspet/oomwoo)

</div>

ROS 2 robot description, Gazebo simulation for **oomwoo-one**, the first
[OOMWOO](https://github.com/makerspet/oomwoo) open-source robot vacuum model. TODO download [3D printing STEP/3MF and CAD design](https://github.com/makerspet/oomwoo-one-cad).

Tutorials:
- [Simulate oomwoo-one in Gazebo with ROS 2](https://makerspet.com/blog/simulate-oomwoo-one-robot-vacuum-in-gazebo-with-ros-2/)
- [Write your first oomwoo ROS 2 package](https://makerspet.com/blog/write-your-first-oomwoo-ros-2-package/)

![Reference robot vacuum cleaner top](https://raw.githubusercontent.com/makerspet/oomwoo/main/assets/vacuum_model_top.webp)

### Video: OOMWOO One Step-by-step Gazebo/ROS2 simulation tutorial
<a href="http://www.youtube.com/watch?feature=player_embedded&v=FOBChivhhkg" target="_blank">
 <img src="http://img.youtube.com/vi/FOBChivhhkg/maxresdefault.jpg" alt="OOMWOO One Step-by-step Gazebo/ROS2 simulation tutorial" width="720" height="405" border="10" />
</a>

## Package contents
- `urdf/` — xacro description of the ~349 mm round vacuum (body + LiDAR turret, diff-drive
  wheels, caster). Frames follow the Kaia.ai convention: `base_footprint → base_link → base_scan`.
- `config/ekf.yaml` — `robot_localization` EKF that fuses `/odom` + `/imu` and publishes the
  `odom → base_footprint` transform (the bridge publishes the `/odom` topic but not this TF,
  which cartographer requires).
- `config/cartographer_lds_2d.lua`, `config/navigation.yaml`, … — SLAM / Nav2 tuning.
- `config/gz_bridge.yaml`, `urdf/plugins.xacro` — Gazebo simulation (diff-drive, odometry,
  gpu_lidar, and front bumper contact sensors). See
  [docs/sim-bumpers.md](docs/sim-bumpers.md) for how the simulated bumpers are wired and
  the three gz-sim gotchas that make them easy to break.
- `launch/bringup.launch.py` — physical bring-up: bridge + `robot_state_publisher` + EKF.

## Usage

Select the robot model (used by the shared Kaia.ai launch files):
```
kaia config robot.model oomwoo_one
```

### Simulation (no robot needed)
```
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run kaiaai_teleop teleop_keyboard
```

### Physical robot
The robot must be on the LAN running SangamIO (see the [Proscenic root &amp; setup tutorial](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/) for flashing/Wi-Fi).
```
ros2 launch oomwoo_one bringup.launch.py robot_ip:=<robot-ip>
ros2 launch kaiaai_bringup navigation.launch.py slam:=True
ros2 run kaiaai_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/maps/map
```
You can store the robot IP once instead of passing `robot_ip:=` every time:
```
kaia config robot.ip <robot-ip>
ros2 launch oomwoo_one bringup.launch.py
```
(Precedence: an explicit `robot_ip:=` wins, otherwise `kaia config robot.ip`, otherwise 192.168.1.143.)

## Notes
- URDF dimensions are approximate (~349 mm diameter, ~95 mm height, 0.233 m wheel base to
  match the bridge's odometry). Refine against measurements of your robot.
- Vacuum-specific actuators (vacuum/brushes/water pump, LEDs) are exposed by the bridge via
  `/set_actuator`, `/set_led`, `/set_lidar` and the `/actuator_cmd`, `/led_cmd` topics.

## License
Apache 2.0
