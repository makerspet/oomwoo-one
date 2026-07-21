# Simulated bumpers (gz-sim contact sensors)

How the front bumper contact sensors work in simulation, why they are easy to
break, and how to verify them. Written after a debugging session where the bumper
topics were silent for **three independent reasons at once** — none of which is
obvious from the ROS or Gazebo side.

The physical robot reports bumps over `/bumper` (`vacuum_ros2_bridge/Bumper`). In
simulation the equivalent is two gz-sim **contact sensors** (`bumper_left`,
`bumper_right`) that watch the front arc-facet collisions and publish
`ros_gz_interfaces/msg/Contacts` on `/bumper_left/contact` and
`/bumper_right/contact`.

## TL;DR — three things must all be true

A gz-sim contact sensor fires **only** when all three hold. Miss any one and the
topic exists but stays completely silent (no error, no warning):

1. **The sensor references the post-lump collision names.** URDF→SDF conversion
   renames every collision; a sensor pointing at the pre-lump name matches nothing.
2. **The bridge subscribes to the sensor's scoped gz topic** — gz-sim ignores the
   sensor's `<topic>` for contact sensors.
3. **The world loads `gz-sim-contact-system`** at world level. The model-level
   plugin is a no-op.

Files involved:
- [`urdf/plugins.xacro`](../urdf/plugins.xacro) — the sensors + collision refs (#1, #3)
- [`urdf/robot.urdf.xacro`](../urdf/robot.urdf.xacro) — the bumper arc-facet collisions
- [`config/gz_bridge.yaml`](../config/gz_bridge.yaml) — the bridge (#2)
- [`test/test_bumper_wiring.py`](../test/test_bumper_wiring.py) — guards #1 and #2

## 1. Collision names change during URDF→SDF conversion

sdformat's URDF parser **lumps** every fixed-joint child link into the canonical
root link (`base_footprint` here) and **renames** each collision to:

```
<root>_fixed_joint_lump__<original_name>_collision_<N>
```

`<N>` is the collision's document-order index within the lump — the **first**
collision (the `base_link` body cylinder) is index 0 and gets *no* suffix; the 12
bumper facets are 1–12. So the facet authored as `bumper_left_0` becomes:

```
base_footprint_fixed_joint_lump__bumper_left_0_collision_1
```

A `<contact><collision>bumper_left_0</collision>` sensor watches a name that no
longer exists → it never reports a contact. This is *the* classic gz-sim/URDF
bumper trap.

`plugins.xacro` therefore emits the post-lump names directly. Because `<N>` is
positional, **reordering or adding `base_link` collisions shifts it and silently
breaks the sensors** — the wiring test catches that.

To regenerate the names after a model change:

```bash
xacro urdf/robot.urdf.xacro | gz sdf -p /dev/stdin | grep 'collision name'
```

> Note: this naming is deterministic and identical whether you convert offline
> (`gz sdf -p`) or let `ros_gz_sim create` convert at spawn time — verified by
> dumping the running world with the `generate_world_sdf` service.

## 2. gz-sim ignores a contact sensor's `<topic>`

The sensors set `<topic>bumper_left/contact</topic>`, but gz-sim's Contact system
publishes contact sensors **only** to the auto-generated scoped name:

```
/world/<world>/model/<model>/link/<root_link>/sensor/<sensor>/contact
```

e.g. `/world/default/model/oomwoo_one/link/base_footprint/sensor/bumper_left/contact`.

So `config/gz_bridge.yaml` bridges that scoped name (not `bumper_left/contact`).
That couples the bridge entry to three names, all currently stable:

| Part | Value | Stable because |
|------|-------|----------------|
| world | `default` | both `test_room.world` and `living_room.world` use it |
| model | `oomwoo_one` | the spawn `-name`; this bridge is oomwoo_one-specific |
| link | `base_footprint` | the canonical root after fixed-joint lumping |

If any of those changes, update the bridge. The wiring test checks the bridge
topic still matches the sensor's actual link.

## 3. The Contact **system** must be world-level

`gz-sim-contact-system` is a world plugin. Declaring it inside the robot model
(`<gazebo><plugin ... Contact/></gazebo>`) is a **no-op** — verified: bumpers fire
in `test_room.world` (which loads it at world level) and stay silent in
`kaiaai_gazebo`'s `living_room.world` (which does not), even though the sensor and
its topic both exist.

Any world used for bumper testing needs, inside `<world>`:

```xml
<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
```

`oomwoo_sim_support/worlds/test_room.world` already has it. **`living_room.world`
(in `kaiaai_gazebo`) does not** — add it there if you need bumpers in that world.

## Verifying bumpers work

Headless, from a built workspace (the `makerspet/oomwoo:jazzy-dev` image):

```bash
# 1. bring up the sim (no Nav2 needed), pinning oomwoo_one
ros2 launch oomwoo_sim_support sim_bringup.launch.py \
    with_nav:=false gui:=false robot_model:=oomwoo_one &

# 2. drive forward into a wall
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' &

# 3. watch the bridged ROS topic — should stream Contacts while pressed
ros2 topic echo /bumper_left/contact ros_gz_interfaces/msg/Contacts
```

A working contact looks like:

```
contacts:
- collision1: { name: oomwoo_one::base_footprint::base_footprint_fixed_joint_lump__bumper_left_0_collision_1 }
  collision2: { name: wall_east::link::c }
  normals: [{ x: -1.0, y: 0.0, z: 0.0 }]     # pointing into the wall
  depths:  [0.0005]
```

Debugging tips when it is silent:

- **Is the robot actually the one you think?** `sim_bringup` reads `robot.model`
  from `kaia config` by default; pin `robot_model:=oomwoo_one` (the image's default
  is a different robot). Check with `gz topic -l | grep sensor/bumper`.
- **gz side vs ROS side.** Echo the raw scoped gz topic to isolate the bridge:
  `gz topic -e -t /world/default/model/oomwoo_one/link/base_footprint/sensor/bumper_left/contact`.
  Data here but not on `/bumper_left/contact` ⇒ a bridge (#2) problem. Silent here
  too ⇒ a sensor (#1) or world-system (#3) problem.
- **Confirm real contact.** If `/odom` stalls against a wall but no contacts fire,
  the physics contact is happening but the sensor is not reporting it — almost
  always #1 or #3.

## Regression test

[`test/test_bumper_wiring.py`](../test/test_bumper_wiring.py) runs
`xacro | gz sdf -p` and asserts (a) every contact-sensor collision ref resolves to
a real collision (#1) and (b) the bridge's scoped topics still match the sensor's
link (#2). It runs under `colcon test` and self-skips where `xacro`/`gz` are not
installed, so it is a no-op on contributor machines without Gazebo.

```bash
colcon test --packages-select oomwoo_one
```

## Geometry note

The bumper is the front 180° of the body wall, modelled as `bumper_facets_per_side`
(6) thin box facets per side (`robot.urdf.xacro`). Each facet's outer face sits
~5 mm proud of the body cylinder, so a wall contacts the facet — and therefore the
sensor's watched collision — before the body. Reducing the facet count or their
protrusion risks the physics solver attributing the contact to the body cylinder
instead, which the bumper sensors do not watch.
