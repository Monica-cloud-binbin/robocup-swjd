# 黄色气球检测 ROS1 package

这个交付包实现 `yellow_detector`，使用 ROS Noetic、Python 3、OpenCV HSV 颜色分割检测 USB 摄像头画面中的最大黄色区域。视觉节点只发布目标信息和调试图像，不发布 MAVROS 指令，也不控制无人机。

## 厂家工程接口核对

已检查你提供的 `my_ws.zip` 中 `src` 下的任务和视觉源码：

- `offboard_circle_modified/src/offboard_circle_modified.cpp` 订阅 `/yellow_balloon_position`，类型为 `geometry_msgs/PointStamped`。其回调把 `point.x` 和 `point.y` 当作气球中心像素坐标，把 `point.z` 限制到 `[0, 1]` 后作为画面面积占比；任务中有黄色目标搜索和接近状态。
- 随包的 `offboard_circle_modified/MISSION_README.md` 写明了相同 topic、消息类型和字段含义，但还称黄色视觉“尚未接入飞控”。这段说明与当前 C++ 源码不一致；实现时以源码中实际存在的订阅、回调和任务状态为准。
- 黑环视觉现有链路是 `my_object_positon` 订阅 `/yolov8/BoundingBoxes`（源码引用 `yolov8_ros_msgs/BoundingBoxes`），发布 `/object_position`（`geometry_msgs/PointStamped`）。消息中的 `point.x` 是中心像素 x，`point.z` 是框高宽比乘 100。黄色 HSV 节点不复用这个黑环 topic。
- `depth_object_position` 使用另一组 YOLO/D435 输入并发布 `/depth_object_position`（`geometry_msgs/PointStamped`）。源码树中没有名为 `object_follow` 或 `object_land` 的 package，也没有 `/yolov8/pub_image_xy` 的引用。任务控制代码是 `offboard_circle_modified`。
- 压缩包的 `src` 中没有 USB 摄像头驱动 package 或 `yolov8_ros_msgs` 的消息定义源码；前者需在你的完整 ROS 环境中提供，后者从厂家已有工程依赖中提供。压缩包内的 mission 源码足以确认黄色飞控 topic 和字段约定。

飞控接口可直接承载中心点和面积占比，所以本节点继续发布原接口 `/yellow_balloon_position`。为满足“是否检测到、bbox、像素面积”的完整结果需求，另增加强类型消息 `yellow_detector/YellowBalloonDetection`，发布于 `/yellow_detector/detection`。`PointStamped` 不包含检测标志、bbox 和原始像素面积，因此不适合作为完整检测记录。

无目标时，`/yellow_detector/detection` 每帧发布 `detected=false`；为符合任务节点现有的目标丢失超时逻辑，`/yellow_balloon_position` 只在成功检测到目标时发布。这样不会把“未检测”伪装成一条刚更新的目标位置。

## Topic 与消息

| Topic | 消息类型 | 内容 |
|---|---|---|
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 默认相机输入，可用 `~image_topic` 改写 |
| `/yellow_balloon_position` | `geometry_msgs/PointStamped` | 飞控接口：`point.x/y` 为框中心像素，`point.z` 为轮廓面积/整帧像素数 |
| `/yellow_detector/detection` | `yellow_detector/YellowBalloonDetection` | 每帧完整状态：检测标志、中心、bbox、轮廓面积和面积占比 |
| `/yellow_detector/debug_image` | `sensor_msgs/Image` | BGR 调试图，检测到时画绿色框、中心点和 `yellow_balloon area=...px` |

`xmin/ymin` 是包围框左上边界；`xmax/ymax` 是右侧和下侧的开区间边界。`area` 是最大轮廓的像素面积，`area_ratio = area / (图像宽 × 图像高)`。HSV 没有模型置信度，因此调试图标注面积。

## 安装和部署

先把交付的 `yellow_detector.zip` 复制到 Ubuntu 电脑（例如 `~/Downloads/yellow_detector.zip`）。压缩包根目录中包含 `yellow_detector/` package 目录。

```bash
# ① 解压完整 package 到工作空间 src
cd ~/cwkj_ws/src
unzip ~/Downloads/yellow_detector.zip

# ② 确认目录里有 package.xml、CMakeLists.txt、msg、scripts 和 launch
ls ~/cwkj_ws/src/yellow_detector

# ③ 给 Python 节点添加可执行权限
chmod +x ~/cwkj_ws/src/yellow_detector/scripts/yellow_detector.py

# ④ 编译工作空间
cd ~/cwkj_ws
catkin_make

# ⑤ source 工作空间
source devel/setup.bash

# ⑥ 启动 USB 摄像头（此 launch 必须已由你的厂家工程安装）
roslaunch usb_cam usb_cam-test.launch

# ⑦ 在另一个终端 source 后启动黄色气球检测
source ~/cwkj_ws/devel/setup.bash
rosrun yellow_detector yellow_detector.py

# ⑧ 在第三个终端查看调试画面
source ~/cwkj_ws/devel/setup.bash
rqt_image_view
```

在 `rqt_image_view` 窗口的 topic 菜单选择 `/yellow_detector/debug_image`。检测成功时应看到绿色矩形框、红色中心点和 `yellow_balloon area=...px` 文本；未检测到时画面会显示 `yellow_balloon: not found`。

也可以用 package 自带 launch 启动检测节点：

```bash
roslaunch yellow_detector yellow_detector.launch
```

如果摄像头实际 topic 不是 `/usb_cam/image_raw`，启动时改 `image_topic` 参数或修改 launch 中的值。先用 `rostopic list` 和 `rostopic type /usb_cam/image_raw` 核对输入 topic。

## 参数调整

阈值是节点启动时读取的 ROS 参数。可在启动节点前设置，例如：

```bash
rosparam set /yellow_detector/h_min 18
rosparam set /yellow_detector/h_max 38
rosparam set /yellow_detector/s_min 80
rosparam set /yellow_detector/s_max 255
rosparam set /yellow_detector/v_min 80
rosparam set /yellow_detector/v_max 255
rosparam set /yellow_detector/min_area 500.0
rosparam set /yellow_detector/morph_kernel_size 5
rosrun yellow_detector yellow_detector.py
```

HSV 的 H 范围为 `0..179`，S/V 为 `0..255`。默认值适合作为黄色目标的起始阈值；现场光照、摄像头白平衡和气球反光会影响分割效果。根据调试图调阈值；更新参数后重启节点使新值生效。`min_area` 按像素计，分辨率改变时通常需要重新设定。

`/yellow_balloon_position.point.z` 的面积占比用于兼容任务代码中的黄色目标距离/接触阈值。当前 detector 使用轮廓面积占整幅图像面积的比例；应使用实际摄像头画面确认它与任务参数中的 `yellow_area_slow_threshold`、`yellow_area_contact_threshold` 一致。

## 调试命令

```bash
# 节点列表
rosnode list

# topic 列表
rostopic list

# 相机输入是否有数据
rostopic hz /usb_cam/image_raw

# 检查飞控兼容接口类型和消息
rostopic type /yellow_balloon_position
rostopic echo /yellow_balloon_position

# 检查完整检测消息（没有目标时 detected 应为 false）
rostopic type /yellow_detector/detection
rostopic echo /yellow_detector/detection
rosmsg show yellow_detector/YellowBalloonDetection

# 查看调试图像
rqt_image_view
```

未检测到时，`/yellow_balloon_position` 不会持续输出；请查看 `/yellow_detector/detection` 中的 `detected` 字段，同时检查摄像头图像和 HSV 参数。节点不会改变飞控、任务状态或飞行控制 topic。
