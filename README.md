See my [main project](https://github.com/slgrobotics/articubot_one/wiki)

-------------------

# map_orientation_saver

A standalone **ROS 2 Lyrical / Python (`ament_python`)** package that estimates
geographic orientation of a local map and saves it beside a Nav2 raster map.
It has no dependency on `articubot_one`, Seggy, Dragger, a Wi-Fi logger, or a
particular SLAM system. Robot-specific integration belongs in the consuming
project. The estimator and sidecar format are adapted from `articubot_one`;
that project is not changed or automatically rewired by installing this package.

The two executables are:

- `map_orientation_node`: continuous, quality-checked IMU/TF orientation estimate.
- `map_saver_with_orientation`: Nav2 map-saving CLI wrapper with an atomic,
  content-hash-bound `<map>.orientation.yaml` sidecar.

## Build and test

```bash
source /opt/ros/lyrical/setup.bash
cd ~/robot_ws
rosdep install --from-paths src/map_orientation_saver --ignore-src -r -y
colcon build --packages-select map_orientation_saver
source install/setup.bash
colcon test --packages-select map_orientation_saver
colcon test-result --verbose
```

Runtime dependencies include `rclpy`, `sensor_msgs`, `std_msgs`, `tf2_ros_py`,
Nav2's map saver, ROS launch/CLI packages, and PyYAML. No custom ROS messages
or services are required. `LICENSE` retains the repository's Apache-2.0 license.

## Start the estimator

Magnetic heading example (the declination is illustrative, not a universal value):

```bash
ros2 launch map_orientation_saver map_orientation.launch.py \
  magnetic_declination_degrees:=4.18
```

Or use a robot-owned configuration file:

```bash
ros2 launch map_orientation_saver map_orientation.launch.py \
  namespace:=robot1 orientation_config_filepath:=/path/to/orientation.yaml
```

```yaml
/**/map_orientation_node:
  ros__parameters:
    imu_topic: sensors/imu
    map_frame: map
    robot_frame: base_link
    heading_reference: magnetic
    magnetic_declination_degrees: 4.18  # Set for your location and IMU configuration.
    yaw_offset: 0.0                    # Heading convention correction, radians.
```

Use `heading_reference: true_enu` only if the upstream IMU already publishes
true-ENU orientation. Do not add declination twice. The default is `magnetic`
with unknown declination (`.nan`), which reports unavailable instead of
silently treating magnetic North as true North. A relative/gyro-only heading
cannot establish geographic orientation. NED data requires full upstream
conversion to ENU, not merely a yaw offset.

Launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `namespace` | empty | ROS namespace for the estimator and relative topics/TF |
| `use_map_orientation_estimator` | `true` | Enable/disable node creation |
| `use_sim_time` | `false` | Use the ROS simulation clock |
| `orientation_config_filepath` | empty | Optional estimator parameter YAML |
| `navsat_config_filepath` | empty | Optional external NavSat parameter YAML |
| `imu_topic`, `map_frame`, `robot_frame` | empty | Override the corresponding YAML values |
| `heading_reference` | empty | Override with `magnetic` or `true_enu` |
| `magnetic_declination_degrees` | empty | Override declination, in degrees |
| `yaw_offset` | empty | Override heading convention correction, in radians |

Precedence is package defaults, explicitly selected NavSat corrections,
explicit estimator YAML, then explicit launch overrides. `use_sim_time` is
always selected by its launch argument. There is no robot-model lookup.

Optional NavSat integration reads **only the file supplied by the caller**:

```bash
ros2 launch map_orientation_saver map_orientation.launch.py \
  navsat_config_filepath:=/path/to/navsat_transform.yaml
```

It recognizes one `navsat_transform` node, including a namespaced or wildcard
node key, converts `magnetic_declination_radians` to the estimator's degrees,
and reuses `yaw_offset` in radians. Ambiguous multiple-node files are rejected.
This reads configuration; it does not depend on or start `robot_localization`.

The node can also run directly:

```bash
ros2 run map_orientation_saver map_orientation_node --ros-args \
  -p magnetic_declination_degrees:=4.18
```

When including the launch from another project, pass only the namespace and
configuration appropriate to that project. If including it inside an existing
namespace group, leave `namespace` empty to avoid applying the namespace twice.
Do not run the old and new estimator together in the same namespace. Existing
`articubot_one` bringup can be started with
`use_map_orientation_estimator:=false` when independently testing this package;
no integration changes to that repository are part of this package.

## Simulation

**A simulation clock alone does not establish a true-heading reference.**
Select the provided simulation override only when the simulated IMU publishes
absolute ENU orientation (as in the existing Seggy Gazebo setup):

```bash
ros2 launch map_orientation_saver map_orientation.launch.py \
  use_sim_time:=true heading_reference:=true_enu
```

The equivalent explicit file is installed as `config/map_orientation_sim.yaml`:

```bash
ros2 launch map_orientation_saver map_orientation.launch.py \
  use_sim_time:=true \
  orientation_config_filepath:="$(ros2 pkg prefix --share map_orientation_saver)/config/map_orientation_sim.yaml"
```

If an external NavSat file is also supplied, the provided simulation file
explicitly clears its declination/yaw offset. For other simulators, configure
and verify their heading conventions instead of assuming them.

## Save a map and orientation sidecar

While the estimator and map publisher are running:

```bash
ros2 run map_orientation_saver map_saver_with_orientation \
  -f ~/launch/maps/backyard \
  --ros-args \
  --params-file "$(ros2 pkg prefix --share map_orientation_saver)/config/map_saver_params.yaml"
```

In simulation add `-p use_sim_time:=true` after `--ros-args` so the saver checks
estimate freshness against the same clock. For a namespaced robot add
`-r __ns:=/robot1`. Nav2 options such as `-t map`, `--fmt pgm`, and ROS parameters
are forwarded; the map publisher and estimator must be reachable in the
selected namespace.

The wrapper invokes `ros2 run nav2_map_server map_saver_cli`; it does not require
a separately running map-saver server or custom map-saving service. It creates:

```text
backyard.yaml
backyard.pgm                  # Or the image format requested from Nav2.
backyard.orientation.yaml
```

Wrapper options:

| Option | Default |
| --- | --- |
| `--orientation-topic` | `localization/map_orientation` |
| `--orientation-timeout` | 3 seconds |
| `--orientation-max-age` | 2 seconds |
| `--orientation-map-frame` | `map` |
| `--save-timeout` | 30 seconds |

Use `-f` with a basename; a trailing `.yaml` is accepted and removed before
calling Nav2. Output directories are created if missing. The wrapper captures
a fresh valid estimate before saving, checks validity/session/segment again at
completion, and writes the sidecar only after Nav2 succeeds and both raster
files can be read. If orientation is unavailable, the raster still saves and
the sidecar records `valid: false` with a reason. A failed Nav2 save writes no
new sidecar and exits nonzero. A sidecar-write failure also exits nonzero but
does not undo an already saved raster.

This command saves occupancy-grid rasters. It does not serialize SLAM Toolbox
posegraphs or Cartographer pbstreams, and does not watch saves made by other
tools. Use this wrapper when you want a matching orientation sidecar.

## Estimate, frames, and quality

The calculation is:

```text
map +X yaw in true ENU = true robot yaw − robot yaw in map
North yaw in map       = 90° − map +X yaw in true ENU
```

The estimator uses IMU source timestamps to look up `map <- robot_frame` and
`imu_frame <- robot_frame` TF, applying full quaternion mounting correction.
It never substitutes a latest-time transform. A bounded queue retains
source-stamped IMU samples while TF catches up, retries without blocking the
executor, and expires samples at `max_age`. Pending new samples do not
invalidate a still-fresh accepted estimate.

Circular averaging handles angle wraparound. The estimate resets after
localization recovery (`localization/recovery_active`), backward source time,
a long input gap, or a large orientation jump. Recovery input is optional;
missing IMU or TF yields unavailable status without affecting localization.

Defaults in `config/map_orientation.yaml`:

- 20 samples minimum, sampled no faster than every 0.2 seconds;
- 30-second averaging window and 1-second maximum source age;
- 25-degree reset jump and 20-degree maximum estimated uncertainty;
- 15-degree assumed IMU uncertainty when its covariance is zero/unknown.

Uncertainty combines circular dispersion with IMU yaw variance, without
assuming consecutive sensor readings are independent. It is not a calibrated
confidence interval. Stable magnetic bias cannot be detected by averaging;
verify against a known reference, particularly indoors.

The reliable transient-local `localization/map_orientation` topic carries a
`std_msgs/String` JSON payload. It reports validity/reason, source ROS timestamp,
publish time, frames, heading reference/correction, session/segment identity,
sample count, geographic orientation, uncertainty, estimate age, and pending-TF
diagnostics. Consumers must check `valid` and freshness. Topics are relative
and launch remaps TF into the selected namespace. Status is published once per
second, including while unavailable. The node does not publish or change TF,
alter localization, or depend on a Wi-Fi database.

## Sidecar format

```yaml
schema_version: 1
map_file: backyard.yaml
map_content_hash: <SHA-256 of YAML bytes, NUL byte, and image bytes>
saved_at: <UTC completion time>
geometry:
  resolution: 0.25
  origin: [-75, -75, 0]
orientation:
  valid: true
  reference: true_north
  map_x_heading_from_east_deg: 12.4
  north_yaw_in_map_deg: 77.6
  uncertainty_deg: 2.1
  # Plus source timestamps, frames, sample count, and diagnostic metadata.
```

These values are examples. Sidecars are replaced atomically and tied to both
map files by content hash, matching the Wi-Fi V2 hash convention. Consumers
must verify that hash before use and account for the raster origin yaw when
drawing North on an image. The estimate describes orientation, not geographic
translation, GNSS position, or altitude. The package does not automatically
make a SLAM map North-up or add compass rendering to another application.
