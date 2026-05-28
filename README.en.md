# ins_gt_logger_ros

ROS1 Noetic logger and RViz publisher for INS-D outdoor ground-truth
inspection. The package uses the bundled Inertial Labs SDK and supports two
INS output packets:

- `opvt2a`: heading/pitch/roll plus position, recommended for normal ROS/RViz
  use.
- `qpvt`: quaternion plus position, useful when the downstream trajectory file
  should preserve the device quaternion directly.

## Configure

Edit the default config:

```bash
/home/sd101t/ins-d/device-driver/my_ins_logger/ins_gt_logger_ros/ins_gt_logger_ros/src/config/ins_gt_logger.yaml
```

Important keys:

```yaml
serial:
  port: /dev/ttyUSB0
  baud: 460800
  packet: opvt2a       # opvt2a or qpvt
  poll_hz: 100.0

record:
  enabled: true
  out_dir: /home/sd101t/ins-d/data/ins_rviz_run
  file_name: ins_raw.csv
  schema: minimal      # minimal or full

convert:
  input_file: ""
  output_file: ""
  output_format: tum
  origin: first_valid
```

`minimal` saves the fields needed for RViz and TUM ground truth. `full` also
saves decoded IMU, magnetic, barometric, GNSS velocity, and latency fields.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd /home/sd101t/ins-d/device-driver/my_ins_logger/ins_gt_logger_ros/ins_gt_logger_ros
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## Run With RViz

Make sure the current shell has serial permissions:

```bash
newgrp dialout
id
ls -l /dev/ttyUSB0
```

Start the publisher:

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch
```

Use a different config file if needed:

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch \
  config:=/home/sd101t/ins-d/device-driver/my_ins_logger/ins_gt_logger_ros/ins_gt_logger_ros/src/config/ins_gt_logger.yaml
```

Run only the publisher:

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch open_rviz:=false
```

Published topics:

- `/ins/odom` (`nav_msgs/Odometry`)
- `/ins/path` (`nav_msgs/Path`)
- `/tf` (`map -> ins_link`) when `ros.publish_tf=true`

The node sets the ENU origin from the first valid latitude/longitude/altitude
sample, then publishes later samples in that local ENU frame.

## Convert To TUM

After recording, generate a TUM trajectory:

```bash
rosrun ins_gt_logger_ros ins_gt_convert \
  --config /home/sd101t/ins-d/device-driver/my_ins_logger/ins_gt_logger_ros/ins_gt_logger_ros/src/config/ins_gt_logger.yaml
```

Override input or output paths:

```bash
rosrun ins_gt_logger_ros ins_gt_convert \
  --config /home/sd101t/ins-d/device-driver/my_ins_logger/ins_gt_logger_ros/ins_gt_logger_ros/src/config/ins_gt_logger.yaml \
  --input /home/sd101t/ins-d/data/ins_rviz_run/ins_raw.csv \
  --output /home/sd101t/ins-d/data/ins_rviz_run/gt.tum
```

TUM layout:

```text
timestamp tx ty tz qx qy qz qw
```

Position is local ENU. For `opvt2a`, the quaternion is computed from
heading/pitch/roll. For `qpvt`, `Lk0` is written as `qw` and `Lk1..Lk3` as
`qx/qy/qz`.

## Path Not Moving In RViz

If attitude changes but the path does not appear, first check whether position
is changing:

```bash
rostopic echo /ins/odom/pose/pose/position
```

Indoor tests often have no valid GNSS/INS position update, so all path points
overlap near the ENU origin. Test outdoors with valid GNSS before judging the
trajectory output.
