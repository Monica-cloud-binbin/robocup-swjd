# 当前主控节点与通信拓扑

本文件按主机工作区 `my_ws/src` 的当前源码和启动文件，以及本机参考目录 `important pkg on cwkj_ws` 中的商家文件整理。它描述**源码计划启动和连接的链路**，不是机载电脑的实时 `rosnode list`。参考目录没有提交到本仓库；商家包在机载电脑上的版本、额外节点和实际传感器话题仍需现场核对。

## 启动组合与节点

通常分别启动以下三组；USB 和 D435 视觉组不能同时启动，因为两者会占用相同的 YOLO 和位置转换节点名/输出话题。S1-F290 的已知实机清单是普通 USB 摄像头、宇树 L1 和 PX4，没有 D435；当前有效链路固定为 USB 组，D435 组仅保留作历史测试参考。

| 启动入口 | 明确启动的 ROS 节点 | 来源与边界 |
| --- | --- | --- |
| [`px4_mavros.launch`](my_ws/src/offboard_circle_modified/launch/px4_mavros.launch) | MAVROS（通常 `/mavros`）、`/laserMapping`、`/tf_pub_1`、`/tf_pub_2`、`/unitree_l1_to_mavros`、`/soundplay_node`、`/voice_server` | 逐层 include 商家 `robot_bringup/launch/px4.launch`、`test_location.launch`、`unitree_lidar_ros` 驱动及本仓库 [`voice.launch`](my_ws/src/my_voice/launch/voice.launch)。雷达驱动、Point-LIO 附加转换节点的具体名称见下文“待核对”。PX4 飞控本身不是 ROS 节点。 |
| [`vision_object_position.launch`](my_ws/src/offboard_circle_modified/launch/vision_object_position.launch) | `/usb_cam`、`/yolov8_ros`、`/my_object_position` | USB 摄像头和 YOLO 的 launch 来自商家 `usb_cam`、`yolov8_ros`；`my_object_position` 统一完成置信度过滤和检测框数据转换。卫星与黄气球共用这一条视觉链。 |
| [`vision_object_position_depth.launch`](my_ws/src/offboard_circle_modified/launch/vision_object_position_depth.launch) | `/d435/realsense2_camera_manager`、`/d435/realsense2_camera`、`/yolov8_ros`、`/my_object_position`、`/yellow_detector` | D435 的 nodelet manager/camera、YOLO 来自商家包；此入口**没有启动** `depth_object_position`。RealSense 可能还加载内部组件。 |
| [`offboard_circle_modified.launch`](my_ws/src/offboard_circle_modified/launch/offboard_circle_modified.launch) | `/offboard_circle_modified` | 主控可执行程序来自 [`offboard_circle_modified.cpp`](my_ws/src/offboard_circle_modified/src/offboard_circle_modified.cpp)。launch 的 `name` 覆盖源码 `ros::init` 使用的默认名字。 |

商家 `px4.launch` 再 include 系统 `mavros/launch/node.launch`，默认飞控串口为 `/dev/ttyTHS0:921600`（MAVLink v2）；这不是在本仓库启动 PX4 SITL。`test_location.launch` 启动 Point-LIO 的 `laserMapping`，并 include `Pointcloud2Map.launch`、`PointsCloud2toLaserscan.launch`。后两个文件及 `unitree_lidar_ros/run_without_rviz.launch` 的源码不在当前参考目录，不能从这份快照确定其中的节点名和原始点云/IMU 话题。

## 主要数据流

```mermaid
flowchart LR
  PX4[PX4 飞控] <-->|MAVLink 串口| MAVROS[MAVROS]
  L1[宇树 L1] --> Driver[Unitree 雷达驱动] --> LIO[Point-LIO /laserMapping]
  LIO -->|/pointlio/odom| Bridge[/unitree_l1_to_mavros]
  Bridge -->|/mavros/vision_pose/pose| MAVROS
  MAVROS -->|/mavros/state; /mavros/local_position/odom| Main[/offboard_circle_modified]
  Main -->|/mavros/setpoint_position/local; arming; set_mode| MAVROS
  Cam[USB 摄像头 /usb_cam] -->|/usb_cam/image_raw| Yolo[/yolov8_ros]
  Yolo -->|/yolov8/BoundingBoxes| Convert[/my_object_position]
  Convert -->|/object_detection| Main
  Convert -.->|/object_position 旧兼容接口| Legacy[旧主控/调试节点]
  Main -->|/play4| Voice[/voice_server] --> Sound[/soundplay_node]
```

雷达原始数据进入 Point-LIO 的具体 ROS 话题未由现有参考源码证实；图中的这一段只表示启动依赖。Point-LIO 到桥接节点的 `/pointlio/odom` 则由商家 `unitree_l1_to_mavros/src/unitree_l1_to_mavros.cpp` 的订阅直接证实。桥接节点把位置和姿态复制成外部视觉位姿交给 MAVROS；PX4 是否融合该位姿还取决于飞控配置，不能仅凭话题存在判定。

### 主控及飞控接口

| 发布者 / 调用者 | 话题或服务 | ROS 类型 | 接收者 / 约定 |
| --- | --- | --- | --- |
| Point-LIO（预期） | `/pointlio/odom` | `nav_msgs/Odometry` | `/unitree_l1_to_mavros` 订阅；实际发布者需在机载确认。 |
| `/unitree_l1_to_mavros` | `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | MAVROS 外部视觉入口；源码直接复制 odom 的位置/姿态，`frame_id` 设为 `map`。 |
| MAVROS | `/mavros/state` | `mavros_msgs/State` | 主控读取连接、模式和解锁状态。 |
| MAVROS | `/mavros/local_position/odom` | `nav_msgs/Odometry` | 主控读取本地位置、姿态，并记录起飞位置；任务坐标参数也应使用这一坐标系。 |
| 主控 | `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 发给 MAVROS 的位置及 yaw 目标；主循环频率 20 Hz。当前源码没有向 MAVROS 速度设定话题发控制指令。 |
| 主控 | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` 服务 | 解锁飞控。 |
| 主控 | `/mavros/set_mode` | `mavros_msgs/SetMode` 服务 | 切换 `OFFBOARD`，任务末尾切换 `AUTO.LAND`。 |
| 主控 | `/play4` | `std_srvs/Trigger` 服务 | `/voice_server` 调用 `sound_play::SoundClient` 播放 `/home/cwkj/4.wav`（绕飞播报）；服务端还提供 `/play1`～`/play3`，但当前主控不调用。 |

`test_location.launch` 还启动两个静态 TF 发布节点：`map -> camera_init`、`aft_mapped -> robot_foot_init`。主控使用的是 MAVROS 里程计消息中的姿态，没有直接订阅 TF 话题。`/soundplay_node` 是 `sound_play` 包的播放节点；其包内传输细节不在本仓库源码中。

### 视觉与任务接口

| 发布者 | 话题 | ROS 类型 | 消费者 / 数据含义 |
| --- | --- | --- | --- |
| `/usb_cam` | `/usb_cam/image_raw` | `sensor_msgs/Image` | `/yolov8_ros` 订阅。商家 `usb_cam/launch/usb_cam-test.launch` 配置 `/dev/video0`、640x480、30 fps；驱动还发布相机信息。 |
| `/yolov8_ros` | `/yolov8/BoundingBoxes` | `yolov8_ros_msgs/BoundingBoxes` | `/my_object_position` 订阅；YOLO 的 `image_topic`、`pub_topic` 在商家 `yolo_v8.launch` 中配置。消息包含 `bounding_boxes[]`，各框有 `Class`、`probability`、`xmin/ymin/xmax/ymax`。 |
| `/my_object_position` | `/object_detection` | `my_object_position/ObjectDetection` | 当前主控的统一视觉接口。转换节点遍历 YOLO 检测框，丢弃低于 `~confidence_threshold`（默认 0.8）及尺寸无效的框，并逐框发布类别、置信度、边界框、中心像素、宽高比和框面积占整幅图像的比例。主控按 launch 中配置的类别名区分卫星与黄气球。 |
| `/my_object_position` | `/object_position` | `geometry_msgs/PointStamped` | 为旧主控保留的兼容接口；每帧只发布第一个通过过滤的框，`point.x/y` 是中心像素，`point.z` 是 `框高/框宽 * 100`。当前主控不再订阅此话题。 |
| `/yolov8_ros` | `/yolov8/detection_image` | `sensor_msgs/Image` | 供图像调试，不是主控的控制接口。 |

统一转换节点源码见 [`my_object_position.cpp`](my_ws/src/my_object_positon/src/my_object_position.cpp)，消息定义见 [`ObjectDetection.msg`](my_ws/src/my_object_positon/msg/ObjectDetection.msg)。新增消息后必须在对应 ROS 工作空间重新编译并 source。旧 `yellow_detector` 包仍保留在源码树中，但 `vision_object_position.launch` 和当前主控已不再使用它。

### D435 替代视觉链

`vision_object_position_depth.launch` 选择商家 `realsense2_camera/rs_camera.launch`，其 `camera` 参数默认 `d435`，所以本组合使用 `/d435/color/image_raw`；商家 `yolo_v8_d435.launch` 也订阅它，向相同的 `/yolov8/BoundingBoxes` 发布。该 D435 启动文件尚未按新的 `/object_detection` 单视觉接口清理，且实机清单没有 D435，因此不能把它当作当前可用链路。

主控源码也订阅 `/depth_object_position`（`geometry_msgs/PointStamped`），但其回调目前只保存 z 值，后续任务逻辑没有读取该变量。两套视觉组合都**没有启动** [`depth_object_position`](my_ws/src/depth_object_position/src/depth_object_position.cpp)。若将来接入，该节点源码当前还存在接口不一致：订阅 `/yolov8/camera_2/BoundingBoxes`，而上述 D435 YOLO 默认发 `/yolov8/BoundingBoxes`；订阅 `/d435/aligned_depth_to_color/image_raw`，而上述 RealSense launch 默认 `align_depth=false`；并且将 `/depth_object_position` 声明为 `PointStamped`，实际发布对象却是 `PoseStamped`。应先修正并在机载验证，不能把它算作当前可用输入。

## 新节点接入前核对

1. 区分图像像素、检测框面积比例、本地里程计米数和姿态；`/object_detection` 的 `center_x/center_y` 是图像像素，不能当作世界坐标。
2. 需要改变无人机运动时，先明确由主控统一发布 `/mavros/setpoint_position/local`，还是设计经主控仲裁的新接口；当前主控源码没有避障输入或仲裁接口。`my_mid360_obstacle_detector` 虽在 `my_ws/src`，上述启动文件均未 include 它，不能把它当作宇树 L1 已接通的避障链。
3. 不要把 `robot_bringup/test_location.launch` 中的“Livox AVIA”旧注释当作机型证据。宇树驱动、Point-LIO 和点云转扫描的具体话题名需在部署环境确认，特别是计划新增避障订阅时。
4. 在机载电脑分别核对 `rosnode list`、`rostopic info /pointlio/odom`、`rostopic info /yolov8/BoundingBoxes`、`rostopic info /object_detection`、`rostopic info /mavros/setpoint_position/local`、`rosservice info /play4` 以及相关话题的 `rostopic type`/`rostopic hz`。本文件只核对了主机源码，没有声称已经验证机载实时拓扑。

商家参考路径均相对于仓库根目录的 `important pkg on cwkj_ws`，该目录被 `.gitignore` 排除，远端读者需另取商家包。相应源码主要有 `robot_bringup/launch/{px4,test_location}.launch`、`usb_cam/launch/usb_cam-test.launch`、`yolov8_ros/yolov8_ros/launch/{yolo_v8,yolo_v8_d435}.launch`、`realsense-ros/realsense2_camera/launch/rs_camera.launch` 和 `unitree_l1_to_mavros/src/unitree_l1_to_mavros.cpp`。
