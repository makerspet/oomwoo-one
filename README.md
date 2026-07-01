# oomwoo-one — ROS 2 robot description + Gazebo simulation

ROS 2 robot description and Gazebo simulation for **oomwoo-one**, the first
[oomwoo](https://github.com/makerspet/oomwoo) open-source robot vacuum model.

> **Forked from the Proscenic M6 Pro** ROS 2 package, with the **LiDAR centered** on the round
> body — use it as the sim baseline for oomwoo software development. Geometry is inherited
> (~349 mm) and will be refined from the sourced parts + 3D design. The physical bring-up files
> (`launch/bringup.launch.py`, the `*bridge*` / `kaiaai.yaml` configs) are inherited
> Proscenic / `vacuum_ros2_bridge` configs — adapt or prune them for oomwoo hardware later.

Tutorial: Step-by-step [root &amp; setup tutorial](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/)

Tutorial: Step-by-step [bringup &amp; SLAM tutorial](https://makerspet.com/blog/tutorial-part-2-drive-map-navigate-your-proscenic-m6-pro-in-ros-2/)

Tutorial: Step-by-step [simulate in Gazebo](https://makerspet.com/blog/simulate-the-proscenic-m6-pro-robot-vacuum-in-gazebo-with-ros-2/)


![Reference robot vacuum cleaner top](https://github.com/makerspet/oomwoo/blob/main/assets/vacuum_model_top.webp)

<img width="1200" height="1200" alt="oomwoo_ros2_gazebo_living_room_simulation" src="https://github.com/user-attachments/assets/0bdb7ca0-746a-4dd5-bb29-26f71b953d6a" />


## Package contents
- `urdf/` — xacro description of the ~349 mm round vacuum (body + LiDAR turret, diff-drive
  wheels, caster). Frames follow the Kaia.ai convention: `base_footprint → base_link → base_scan`.
- `config/ekf.yaml` — `robot_localization` EKF that fuses `/odom` + `/imu` and publishes the
  `odom → base_footprint` transform (the bridge publishes the `/odom` topic but not this TF,
  which cartographer requires).
- `config/cartographer_lds_2d.lua`, `config/navigation.yaml`, … — SLAM / Nav2 tuning.
- `config/gz_bridge.yaml`, `urdf/plugins.xacro` — Gazebo simulation (diff-drive, odometry,
  gpu_lidar) — simulation does **not** use the bridge.
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
The robot must be on the LAN running SangamIO (see the tutorial for flashing/Wi-Fi).
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
