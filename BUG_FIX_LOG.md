# Bug Fix Log

记录已确认的问题，优先保持简短、易读、准确。按“功能现象异常”和“终端/节点报错”分组。每条只保留关键表现、原因、可复用修复步骤和验证状态；修复步骤不重复解释原因，没有必要的预防说明不写。是否解决或实际采用的修复不明确时，先问用户；未验证的记录标为待验证。

## 功能现象异常

### 2026-10-02：`yellow_detector/debug_image` 无画面（测试环境已解决；机载端待验证）

- **表现**：`/usb_cam/image_raw` 有画面，`/yellow_detector/debug_image` 无画面。
- **原因**：检测节点订阅的话题与 USB 摄像头发布话题不一致。用户在相机终端补做与检测终端相同的 ROS/工作空间 `source` 后也解决了问题；具体是哪项环境变量造成影响未记录。
- **修复**：将 `yellow_detector.launch` 的 `image_topic` 设为 `/usb_cam/image_raw`；每个启动终端分别 source 相同的 ROS 和工作空间环境后重启节点。
- **验证**：用户确认测试环境中 debug 图像可见。更改尚未同步到机载电脑。

### 2026-10-01：识别到黑环但 `/object_position` 无消息（已解决）

- **表现**：YOLO 报告识别到黑环，但下游收不到 `/object_position`。
- **原因**：消息接口修改后未完整重编译，发布端与坐标转换节点使用的生成接口不一致。
- **修复**：统一 `.msg` 定义；在包含发布端、订阅端和消息包的工作空间重新编译，source 后重启节点。
- **验证**：用户确认统一接口并重新编译后恢复。

## 终端/节点报错

### 2026-09-30：`yellow_detector.msg` 导入失败（已解决）

- **报错**：`ModuleNotFoundError: No module named 'yellow_detector.msg'; 'yellow_detector' is not a package`。
- **原因**：名为 `yellow_detector.py` 的节点脚本与 ROS 包同名，遮蔽了 `yellow_detector.msg` 模块。
- **修复**：将脚本改名为 `yellow_detector_node.py`，并同步更新 CMake 和 launch 中的引用。
- **验证**：节点成功导入消息并启动。

### 2026-09-30：脚本改名后仍执行旧 devel 包装器（已解决）

- **报错**：旧包装器尝试打开已改名的 `src/yellow_detector/scripts/yellow_detector.py`，出现 `FileNotFoundError`。
- **原因**：`devel/lib/yellow_detector/` 中残留旧生成包装器。
- **修复**：确认路径后删除旧包装器 `~/my_ws/devel/lib/yellow_detector/yellow_detector.py`；在 `~/my_ws` 重新运行 `catkin_make` 并 source `devel/setup.bash`。
- **验证**：新节点脚本可由 launch 启动，旧错误消失。

### 2026-09-30：独立运行节点时 ROS master 不可用（已解决）

- **报错**：`Unable to register with master node [...]`；Ctrl-C 后出现 `ROSInitException: Failed to initialize time`。
- **原因**：`ROS_MASTER_URI` 指向的 master 未运行或不可达；后一个异常是初始化失败后的连带信息。
- **修复**：先启动对应的 `roscore` 或系统 launch，并确认 `echo $ROS_MASTER_URI` 指向该 master。
- **验证**：完整视觉 launch 成功启动 master 和节点。

### 2026-09-30：主视觉 launch 找不到 `usb_cam`（已解决）

- **报错**：`Resource not found: usb_cam`。
- **原因**：构建 `my_ws` 时没有把提供 `usb_cam`/`yolov8_ros` 的 `cwkj_ws` 配置为 underlay。
- **修复**：
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/cwkj_ws/devel/setup.bash
  cd ~/my_ws && catkin_make --force-cmake
  source ~/my_ws/devel/setup.bash
  rospack find usb_cam
  rospack find yolov8_ros
  ```
- **验证**：两个包均解析到预期的 `cwkj_ws` 路径，视觉 launch 成功启动相机。

### 2026-10-01：`BoundingBoxes` 没有 `len` 属性（已解决）

- **报错**：`AttributeError: 'BoundingBoxes' object has no attribute 'len'`。
- **原因**：`my_ws` 与 `cwkj_ws` 的 `BoundingBoxes` 消息定义不一致，且运行时生成代码未随接口更新。
- **修复**：统一两处 `.msg` 定义；重新编译使用该消息的工作空间，并在运行终端 source 正确的工作空间后重启节点。
- **验证**：用户确认统一接口并重新编译后解决。
