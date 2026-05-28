# ins_gt_logger_ros

这是一个面向 INS-D 传感器的 ROS1 Noetic 记录、可视化和离线真值转换包。主要用途是把室外实验中 INS/GNSS 输出的位置和姿态作为激光 SLAM 评估用的近似真值。

英文说明见 [README.en.md](README.en.md)。

## 功能

- 通过 Inertial Labs SDK 连接 INS-D 串口。
- 支持 `opvt2a` 和 `qpvt` 两种输出格式。
- 实时发布 `/ins/odom`、`/ins/path`，可在 RViz 中查看姿态和轨迹。
- 记录 SDK 解码后的 CSV 文件。
- 离线把 CSV 转换为 TUM 轨迹文件。

推荐默认使用 `opvt2a`：它包含 `Heading/Pitch/Roll + Latitude/Longitude/Altitude`，适合 ROS 可视化和调试。`qpvt` 包含四元数 `Lk0..Lk3`，适合需要直接保留传感器四元数的后续评估。

## 配置文件

默认配置文件：

```bash
config/ins_gt_logger.yaml
```

示例：

```yaml
serial:
  port: /dev/ttyUSB0
  baud: 460800
  packet: opvt2a
  poll_hz: 100.0

ros:
  frame_id: map
  child_frame_id: ins_link
  publish_tf: true
  path_max_size: 20000

record:
  enabled: true
  out_dir: data/ins_rviz_run
  file_name: ins_raw.csv
  schema: minimal

convert:
  input_file: ""
  output_file: ""
  output_format: tum
  origin: first_valid
```

## 配置参数说明

### `serial`

| 参数 | 含义 | 可选项/示例 |
| --- | --- | --- |
| `port` | 传感器串口设备路径。 | `/dev/ttyUSB0`, `/dev/ttyUSB1` |
| `baud` | 串口波特率，需要和设备设置一致。 | 推荐 `460800` |
| `packet` | 请求传感器输出的数据格式。 | `opvt2a`, `qpvt` |
| `poll_hz` | ROS 节点读取并发布数据的频率。 | `50.0`, `100.0` |

### `ros`

| 参数 | 含义 | 可选项/示例 |
| --- | --- | --- |
| `frame_id` | 发布 odom/path 使用的全局坐标系名称。 | 默认 `map` |
| `child_frame_id` | INS 机体系 frame 名称。 | 默认 `ins_link` |
| `publish_tf` | 是否发布 `frame_id -> child_frame_id` 的 TF。 | `true`, `false` |
| `path_max_size` | `/ins/path` 最多保留多少个轨迹点，防止 RViz 长时间运行占用过多内存。`0` 表示不裁剪。 | 默认 `20000` |

### `record`

| 参数 | 含义 | 可选项/示例 |
| --- | --- | --- |
| `enabled` | 是否记录 CSV 文件。 | `true`, `false` |
| `out_dir` | 记录文件输出目录。相对路径会以运行 `roslaunch` 或 `rosrun` 时的当前目录为基准。 | `data/ins_rviz_run`, `/tmp/ins_rviz_run` |
| `file_name` | 记录文件名。 | 默认 `ins_raw.csv` |
| `schema` | CSV 保存字段集合。 | `minimal`, `full` |

`schema` 说明：

- `minimal`：保存后续真值转换和 RViz 可视化需要的核心字段，包括时间戳、经纬高、速度、四元数、欧拉角、GNSS 经纬高。
- `full`：在 `minimal` 基础上额外保存陀螺仪、加速度计、磁力计、温度、电压、气压/气压高度、GNSS 速度和延迟字段。用于排错和后续分析，但文件更大。

### `convert`

| 参数 | 含义 | 可选项/示例 |
| --- | --- | --- |
| `input_file` | 离线转换输入 CSV。为空时默认使用 `record.out_dir/record.file_name`。 | `""`, `/path/to/ins_raw.csv` |
| `output_file` | 离线转换输出真值文件。为空时默认使用 `record.out_dir/gt.tum`。 | `""`, `/path/to/gt.tum` |
| `output_format` | 输出真值文件格式。 | 当前仅支持 `tum` |
| `origin` | 局部坐标原点选择方式。 | 当前仅支持 `first_valid` |

`first_valid` 表示使用第一帧有效 `Latitude/Longitude/Altitude` 作为 ENU 局部坐标原点，后续所有经纬高都转换到这个局部坐标系下。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd path/to/ins_gt_logger_ros_workspace
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## 实时运行和 RViz 可视化

确认当前 shell 有串口权限：

```bash
newgrp dialout
id
ls -l /dev/ttyUSB0
```

使用默认配置启动：

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch
```

指定其他配置文件：

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch \
  config:=config/ins_gt_logger.yaml
```

只启动发布节点，不打开 RViz：

```bash
roslaunch ins_gt_logger_ros ins_gt_rviz.launch open_rviz:=false
```

发布话题：

- `/ins/odom`：`nav_msgs/Odometry`
- `/ins/path`：`nav_msgs/Path`
- `/tf`：当 `ros.publish_tf=true` 时发布 `map -> ins_link`

轨迹坐标逻辑：节点读取每帧 `Latitude/Longitude/Altitude`，用第一帧有效经纬高初始化 ENU 原点，然后把后续经纬高转换为局部 `East/North/Up`，对应 ROS 位置 `x/y/z`。

## 离线转换为 TUM

采集完成后，使用同一个配置文件生成 TUM 轨迹：

```bash
rosrun ins_gt_logger_ros ins_gt_convert \
  --config config/ins_gt_logger.yaml
```

也可以临时覆盖输入输出路径：

```bash
rosrun ins_gt_logger_ros ins_gt_convert \
  --config config/ins_gt_logger.yaml \
  --input data/ins_rviz_run/ins_raw.csv \
  --output data/ins_rviz_run/gt.tum
```

TUM 输出格式：

```text
timestamp tx ty tz qx qy qz qw
```

- `tx ty tz`：局部 ENU 坐标，分别对应 East、North、Up。
- `opvt2a`：由 `Heading/Pitch/Roll` 计算四元数。
- `qpvt`：按 `Lk0 -> qw`，`Lk1/Lk2/Lk3 -> qx/qy/qz` 输出。

## RViz 中姿态动但轨迹不动

如果移动传感器时姿态正常，但 `/ins/path` 看不到明显轨迹，先检查位置是否变化：

```bash
rostopic echo /ins/odom/pose/pose/position
```

室内通常没有可靠 GNSS/INS 位置更新，经纬度可能保持不变，所以所有 path 点会重叠在 ENU 原点附近。需要在室外开阔环境、GNSS 有效后再判断轨迹输出是否正常。
