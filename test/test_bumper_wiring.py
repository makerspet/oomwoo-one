# Copyright 2026 makerspet
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
"""
Guards the sim bumper wiring, which has three fragile couplings to gz-sim's
URDF->SDF behaviour (see urdf/plugins.xacro and config/gz_bridge.yaml):

  1. Fixed-joint lumping renames every collision to
     <root>_fixed_joint_lump__<name>_collision_<N>, so the contact sensors must
     reference those post-lump names. If base_link collisions are reordered, N
     shifts and the sensors silently stop firing.
  2. gz-sim ignores a contact sensor's <topic> and publishes only to the scoped
     name /world/<world>/model/<model>/link/<root>/sensor/<sensor>/contact, which
     the bridge must subscribe to verbatim.

This test converts the xacro the same way gz does (xacro | gz sdf -p) and asserts
both couplings still hold. It skips where xacro/gz are unavailable so it is a
no-op on contributor machines without a Gazebo install.
"""
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

try:
    import yaml
except ImportError:
    yaml = None

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(PKG, 'urdf', 'robot.urdf.xacro')
BRIDGE = os.path.join(PKG, 'config', 'gz_bridge.yaml')
MODEL_NAME = 'oomwoo_one'   # the spawn -name the bridge scoped topics assume


def _to_sdf():
    """xacro -> URDF -> SDF exactly as gz does when spawning."""
    if not shutil.which('xacro') or not shutil.which('gz'):
        pytest.skip('xacro/gz not on PATH; bumper wiring test needs a Gazebo install')
    urdf = subprocess.check_output(['xacro', XACRO], text=True)
    path = None
    try:
        with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
            f.write(urdf)
            path = f.name
        return subprocess.check_output(
            ['gz', 'sdf', '-p', path], text=True, stderr=subprocess.DEVNULL)
    finally:
        if path:
            os.unlink(path)


def test_contact_sensor_collision_refs_resolve():
    """Every <contact><collision>NAME</collision> must name a real collision."""
    sdf = _to_sdf()
    real = set(re.findall(r'<collision name=[\'"]([^\'"]+)', sdf))
    refs = re.findall(r'<collision>([^<]+)</collision>', sdf)
    assert refs, 'no contact-sensor collision refs found (are the bumper sensors present?)'
    missing = sorted(r for r in refs if r not in real)
    assert not missing, (
        'contact sensor references collisions that do not exist after URDF->SDF '
        'lumping (bumpers will be silent): {}\nregenerate names with: '
        'xacro robot.urdf.xacro | gz sdf -p /dev/stdin | grep "collision name"'
        .format(missing))


def test_bridge_topics_match_sensor_scoped_names():
    """The bridge's bumper gz topics must equal each sensor's real scoped name."""
    if yaml is None:
        pytest.skip('pyyaml not available')
    sdf = _to_sdf()
    root = ET.fromstring(sdf)
    # which link does gz attach each contact sensor to (after lumping)?
    sensor_link = {}
    for link in root.iter('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type') == 'contact':
                sensor_link[sensor.get('name')] = link.get('name')
    assert {'bumper_left', 'bumper_right'} <= set(sensor_link), \
        'expected bumper_left/bumper_right contact sensors, found {}'.format(sensor_link)

    with open(BRIDGE) as f:
        entries = yaml.safe_load(f)
    by_ros = {e['ros_topic_name']: e['gz_topic_name']
              for e in entries if 'bumper' in e['ros_topic_name']}
    for side in ('left', 'right'):
        gz = by_ros.get('bumper_{}/contact'.format(side))
        assert gz, 'no bridge entry for bumper_{}/contact'.format(side)
        link = sensor_link['bumper_{}'.format(side)]
        want = '/model/{}/link/{}/sensor/bumper_{}/contact'.format(MODEL_NAME, link, side)
        assert gz.endswith(want), \
            'bridge gz_topic_name {!r} no longer matches the sensor scoped name (...{})'\
            .format(gz, want)
