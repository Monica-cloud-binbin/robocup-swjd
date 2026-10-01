# 实机调试说明

本工作区保留了原来的 ROS1、MAVROS、位置期望发布、黑环话题、三路语音服务和圆周控制方式，并在 `offboard_circle_modified` 中增加了最小任务状态机。

## 状态编号

| 编号 | 状态 | 完成条件 |
|---:|---|---|
| 0 | TAKEOFF | 到达起飞高度且 OFFBOARD、解锁状态稳定 3 s |
| 1 | TRANSIT_START_TO_SATELLITE | 走完启动区到卫星搜索圆入口的航点 |
| 2 | SATELLITE_SEARCH | 黑环在允许区间内连续达到配置次数 |
| 3 | SATELLITE_CAPTURE | 保持 `satellite_capture_duration`；黑环长时间丢失则返回搜索 |
| 4 | SATELLITE_ORBIT | 保持 `satellite_orbit_duration`；黑环长时间丢失则返回搜索 |
| 5 | TRANSIT_SATELLITE_TO_YELLOW | 走完卫星到黄色搜索点的航点 |
| 6 | YELLOW_SEARCH | 黄色消息连续达到配置次数 |
| 7 | YELLOW_APPROACH | 黄色面积比例达到接触阈值 |
| 8 | YELLOW_CONTACT_SPIN | 自转接触计时完成 |
| 9 | TRANSIT_YELLOW_TO_HOME | 走完返航航点 |
| 10 | LAND | AUTO.LAND 完成并上锁 |
| 11 | COMPLETE | 任务结束 |

## 现场必须检查

1. 修改 `launch/offboard_circle_modified.launch` 中卫星、黄色搜索点和红球坐标。坐标系原点为启动位置，x 向初始机头前方，y 向左，z 向上。
2. 三个绕行参数只允许 `NONE`、`LEFT`、`RIGHT`。启用 LEFT/RIGHT 前必须确认起点和终点都在红球安全圆外；否则程序会报警并原地保持。
3. `hook_safe_radius` 是无人机机体中心轨迹到红球中心的水平距离；需要把气球半径、无人机外廓和安全余量一并考虑。
4. 四个卫星半径同样是机体中心半径，不能直接照搬规则中的装置间距。
5. 黄色视觉尚未包含在工作区中。未来节点应发布 `/yellow_balloon_position`（`geometry_msgs/PointStamped`）：x 为水平像素，y 为垂直像素，z 为 0.0～1.0 的画面面积比例。没有发布者时程序会留在黄色搜索状态，不会崩溃。
6. `play1`、`play2`、`play3` 分别在卫星捕获、卫星绕飞、黄色接触动作进入时调用一次。请确认三个 wav 文件的内容与现场播报要求一致。

## 编译与启动

在 ROS1 机器上进入工作区后执行：

```bash
catkin_make
source devel/setup.bash
roslaunch offboard_circle_modified offboard_circle_modified.launch
```

压缩包中原有的 `build/`、`devel/` 是其他机器生成的缓存，不应跨机器复用；本交付目录已把它们移到 `legacy_build_cache/`，因此 `catkin_make` 会重新生成干净的顶层 `build/` 和 `devel/`。确认新编译正常后，可自行删除 `legacy_build_cache/`。

## 保持不变的部分

- 黑环话题仍为 `/object_position`，消息规格不变。
- 三路语音服务仍为 `play1`、`play2`、`play3`。
- MAVROS 状态、解锁、OFFBOARD、AUTO.LAND 和位置期望话题保持原写法。
- `Kp_fast`、`Kp_slow`、`Kd`、`aspect_threshold`、`delta_omega` 的黑环角速度反馈逻辑保留。
- 没有增加动态红球视觉、A*/RRT、多障碍物规划、目标卫星停靠或复杂 ABORT 系统。
