#!/usr/bin/env python3
#
# Copyright 2024-2025 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Physical bring-up for the Proscenic M6 Pro robot vacuum.
#
# Unlike the ESP32 + micro-ROS robots (oomwoo_bringup/physical.launch.py, which
# starts a micro_ros_agent + kaiaai_telemetry/telem), the Proscenic runs SangamIO
# on-board and is bridged to ROS 2 over TCP by remakeai/vacuum_ros2_bridge.
#
# This launch starts:
#   1. vacuum_ros2_bridge  - connects to SangamIO (robot_ip:5555); publishes
#                            /scan, /odom, /imu, /battery, ...; subscribes /cmd_vel.
#                            Frames are reconciled to the kaiaai/URDF convention
#                            (base_footprint / base_scan) so SLAM/Nav work unchanged.
#   2. robot_state_publisher - publishes the URDF and the static TF tree
#                            (base_footprint -> base_link -> base_scan, wheels, ...).
#   3. robot_localization EKF - fuses /odom + /imu and publishes the
#                            odom -> base_footprint transform that the bridge does
#                            NOT provide and that cartographer requires
#                            (its config has provide_odom_frame = false).
#
# Usage:
#   ros2 launch oomwoo_one bringup.launch.py robot_ip:=<robot-ip>

import os
import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from kaiaai import config


def make_nodes(context: LaunchContext, robot_model, robot_ip, use_sim_time, use_ekf,
               robot_port, scan_time_offset, scan_mask_deg):
    robot_model_str = context.perform_substitution(robot_model)
    robot_ip_str = context.perform_substitution(robot_ip)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    use_ekf = context.perform_substitution(use_ekf).lower() == 'true'
    robot_port_str = context.perform_substitution(robot_port)
    scan_time_offset_arg = context.perform_substitution(scan_time_offset)
    scan_mask_deg_arg = context.perform_substitution(scan_mask_deg)

    if len(robot_model_str) == 0:
        robot_model_str = config.get_var('robot.model')

    # Robot IP precedence: explicit robot_ip:= arg > 'kaia config robot.ip' > default.
    if len(robot_ip_str) == 0:
        robot_ip_str = config.get_var('robot.ip') or '192.168.1.143'

    description_package_path = get_package_share_path(robot_model_str)

    urdf_path_name = os.path.join(
        description_package_path, 'urdf', 'robot.urdf.xacro')
    ekf_path_name = os.path.join(
        description_package_path, 'config', 'ekf.yaml')
    bridge_launch_path_name = os.path.join(
        get_package_share_path('vacuum_ros2_bridge'),
        'launch', 'bridge.launch.py')

    # Robot-specific bridge tuning (scan_time_offset, scan_mask_deg) lives in
    # config/vacuum_bridge.yaml so it stays out of the launch command. A non-empty
    # command-line arg still overrides the file (handy for live calibration).
    vacuum_bridge_yaml = os.path.join(
        description_package_path, 'config', 'vacuum_bridge.yaml')
    bridge_cfg = {}
    try:
        with open(vacuum_bridge_yaml) as f:
            doc = yaml.safe_load(f) or {}
        for section in doc.values():
            if isinstance(section, dict) and 'ros__parameters' in section:
                bridge_cfg = section['ros__parameters']
                break
    except FileNotFoundError:
        pass
    scan_time_offset_str = scan_time_offset_arg or str(bridge_cfg.get('scan_time_offset', 0.0))
    scan_mask_deg_str = scan_mask_deg_arg or str(bridge_cfg.get('scan_mask_deg', ''))

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path_name]), value_type=str)

    use_sim_time = use_sim_time_str.lower() == 'true'

    print('URDF file   : {}'.format(urdf_path_name))
    print('EKF params  : {}'.format(ekf_path_name))
    print('Bridge launch: {}'.format(bridge_launch_path_name))
    print('Robot IP    : {}'.format(robot_ip_str))
    print('Use EKF     : {}'.format(use_ekf))

    nodes = [
        # 1. SangamIO <-> ROS 2 bridge. Frame IDs reconciled to the kaiaai
        #    convention: odom child = base_footprint, LiDAR frame = base_scan.
        #    When the EKF is off, the bridge broadcasts odom -> base_footprint
        #    itself (publish_tf); when on, the EKF owns that transform.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch_path_name),
            launch_arguments={
                'robot_ip': robot_ip_str,
                'robot_port': robot_port_str,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'lidar_frame_id': 'base_scan',
                'publish_tf': 'false' if use_ekf else 'true',
                'scan_time_offset': scan_time_offset_str,
                'scan_mask_deg': scan_mask_deg_str,
            }.items()
        ),
        # 2. URDF + static TF tree.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),
        # 2b. Publish /joint_states for the (continuous) wheel joints. In Gazebo the
        #     sim plugin publishes these; on the physical robot nothing does, so
        #     robot_state_publisher cannot compute wheel_left/right_link transforms
        #     ("No transform to odom" in RViz). The wheels are spun for display only
        #     (held at 0); their motion is captured by /odom, not these joints.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),
    ]

    # 3. EKF (optional, OFF by default). When enabled it fuses odometry and
    #    publishes odom -> base_footprint. It is off by default while IMU fusion
    #    is on hold: the bridge provides that TF straight from wheel odometry,
    #    which is simpler to debug (no EKF velocity-integration layer). Re-enable
    #    with use_ekf:=true once the IMU is calibrated and worth fusing.
    if use_ekf:
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_path_name, {'use_sim_time': use_sim_time}]
        ))

    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='',
            description='Robot description package name'
        ),
        DeclareLaunchArgument(
            name='robot_ip',
            default_value='',
            description="Proscenic vacuum IP (SangamIO). If empty, uses "
                        "'kaia config robot.ip', else 192.168.1.143."
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='use_ekf',
            default_value='false',
            choices=['true', 'false'],
            description='Run the robot_localization EKF for odom->base_footprint. '
                        'Off by default: the bridge publishes that TF from wheel '
                        'odometry. Enable once IMU fusion is calibrated.'
        ),
        DeclareLaunchArgument(
            name='robot_port',
            default_value='5555',
            description='SangamIO TCP port. Change when port-forwarding several '
                        'vacuums through one firewall on distinct ports.'
        ),
        DeclareLaunchArgument(
            name='scan_time_offset',
            default_value='',
            description='Override config/vacuum_bridge.yaml scan_time_offset (s). '
                        'Empty = use the value in that file.'
        ),
        DeclareLaunchArgument(
            name='scan_mask_deg',
            default_value='',
            description='Override config/vacuum_bridge.yaml scan_mask_deg (deg). '
                        'Empty = use the value in that file.'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('robot_model'),
            LaunchConfiguration('robot_ip'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('use_ekf'),
            LaunchConfiguration('robot_port'),
            LaunchConfiguration('scan_time_offset'),
            LaunchConfiguration('scan_mask_deg'),
        ]),
    ])
