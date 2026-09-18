# OD_ws

基于 ROS 1 的 OpenDRIVE 地图与参考线示例工作区。项目包含自定义 ROS 消息、OpenDRIVE 车道离散化代码，以及根据轨迹请求和车辆定位发布前方参考点的 Python 节点。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/common_msgs`、`src/device_msgs`、`src/od_map_msgs`、`src/perception_msgs` | catkin 消息包，生成节点所需的消息类型 |
| `routeplan/main.py` | 路径规划节点入口 |
| `routeplan/opendrive2discretenet/` | OpenDRIVE 解析和车道离散化 |
| `routeplan/referenceline/` | 轨迹点匹配及参考点信息生成 |
| `routeplan/changda_centerpoint.xodr` | 当前节点使用的示例地图 |
| `routeplan_old/` | 保留的旧版实现，当前运行入口不依赖它 |

## 环境与构建

代码使用 Python 3 和 ROS 1 `rospy`；当前工作区使用 ROS Noetic。除 ROS 自带的 `rospy` 和消息生成工具外，Python 代码还使用 `numpy`、`scipy`、`lxml`、`pyproj`。消息包依赖 `std_msgs`、`geometry_msgs` 和 `sensor_msgs`。

在 Ubuntu 20.04 / ROS Noetic 环境中，可安装对应依赖后构建消息包：

```bash
sudo apt install ros-noetic-catkin ros-noetic-rospy ros-noetic-std-msgs \
  ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
  python3-numpy python3-scipy python3-lxml python3-pyproj
source /opt/ros/noetic/setup.bash
cd /path/to/OD_ws
catkin_init_workspace src
catkin_make
source devel/setup.bash
```

`build/` 和 `devel/` 是本地构建产物，不纳入版本控制。克隆后需在本机重新构建并执行 `source devel/setup.bash`。

## 运行

先启动 ROS master，再从 `routeplan/` 目录启动节点。代码中的地图文件名是相对路径 `changda_centerpoint.xodr`，因此运行目录必须包含该文件。

```bash
# 终端 1
source /opt/ros/noetic/setup.bash
roscore

# 终端 2
cd /path/to/OD_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd routeplan
python3 main.py
```

节点在启动时解析示例 OpenDRIVE 地图，订阅轨迹点和定位消息，并发布约 80 米范围内的前方参考点。主要 ROS 话题如下：

| 方向 | 话题 | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/test_trajectory_req` | `device_msgs/participantTrajectories` | 接收离散轨迹点 |
| 订阅 | `/cicv_location` | `perception_msgs/PerceptionLocalization` | 接收车辆定位 |
| 发布 | `/test_trajectory_result` | `device_msgs/startTask` | 收到轨迹请求后发布任务消息 |
| 发布 | `/OdMap` | `od_map_msgs/OdMap` | 发布含 `routing_points` 的参考线 |

## 当前实现的约束

- 地图路径和部分地图对象索引写在 `routeplan/opendrive2discretenet/discretelanes.py` 中，直接替换地图可能需要调整相关代码。
- `PathPointProvider.py` 使用固定的 UTM 坐标偏移量（`307000`、`3805000`），与当前示例地图绑定；使用其他地区地图时需修改坐标处理。
- 当前节点只有在收到轨迹请求并生成参考点后，才会随定位更新发布 `/OdMap`。地图解析和点匹配需要有效的示例地图及对应格式的输入数据。
