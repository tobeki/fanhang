# 无人机手动/任务模式返航路径规划项目------背景、总体方案与 AI 交接说明

> **文档用途**：本文件用于项目长期交接。后续若更换 ChatGPT / Codex /
> 其他 AI 会话，请优先将本文件提供给新的
> AI，并要求其在修改代码前完整阅读本文件和
> `src/manual_return_planner/README.md`。
>
> **当前文档版本**：V0.1\
> **当前项目阶段**：Manual Return V1.0
> 已完成算法与离线/可视化验证；下一阶段为 V1.0.1"单控制源接管 + ROS
> 实际闭环返航"。\
> **最后更新**：2026-08-20

------------------------------------------------------------------------

# 1. 项目背景

公司已有成熟且销量较好的室内巡检无人机。无人机通过预设航点（Waypoint）和
Action 执行自动巡检任务。

该无人机：

-   不依赖 GPS；
-   依靠激光雷达实时扫描环境；
-   使用公司自研定位算法完成室内定位；
-   能够在大型室内空间完成高精度定位与自动飞行；
-   当前暂不具备真正的实时绕障返航能力；
-   遇到即将碰撞障碍物等情况时，现有系统可以触发暂停飞行等上层 Action。

老板当前提出的新需求是：

> 当无人机在飞行过程中因通信中断、遇到无法继续前进的障碍物或其他外部原因触发返航后，返航模块根据无人机已经飞过的历史轨迹和已扫描地图，规划一条安全返航路径，使无人机尽可能沿已经验证过的飞行区域返回原点（通常为起点/Home）。

触发条件本身不属于本项目范围。

本项目只负责：

``` text
外部返航触发
    ↓
获取已飞历史轨迹
    ↓
生成返航路径
    ↓
将返航轨迹交给现有运动控制系统执行
    ↓
返回 Home
```

------------------------------------------------------------------------

# 2. 老板提出的三阶段总体目标

## 2.1 第一版：历史轨迹原路回溯

最基础版本：

``` text
历史10Hz轨迹
    ↓
反向
    ↓
轨迹压缩/平滑
    ↓
原路返回
```

无人机实际飞行轨迹通过约 10 Hz
频率采样记录，因此一段几十秒的飞行可能包含数百甚至上千个轨迹点。

第一版的主要工作不是重新寻找一条路，而是：

1.  清洗历史轨迹；
2.  删除由于 10 Hz 高频采样产生的几何冗余点；
3.  反转轨迹；
4.  在不破坏原路径空间形态的前提下大幅减少航点；
5.  后续加入点云安全验证；
6.  将处理后的轨迹交给现有控制器执行。

第一版必须坚持：

> 可以删除"采样冗余"，但不能随意删除"空间路线"。

例如无人机从主路线拐入一个拍照区域后又返回主路线，这段支路在 V1
中必须保留。

## 2.2 第二版：冗余路线优化

V2 在 V1 基础上允许删除真正不必要的飞行路线，例如：

-   在巡检点长时间停留产生的重复区域；
-   从主路线进入支路拍照后又原路退出的往返支路；
-   明显重复绕行；
-   其他不影响安全返回的历史冗余路线。

V2 的本质是：

> 从"严格原路返回"升级为"尽量沿已知安全区域，但允许优化不必要的路线"。

V2 暂未开发。

## 2.3 第三版：实时动态返航规划

V3 在性能允许的情况下加入：

-   实时无人机位置；
-   实时局部地图；
-   新出现障碍物；
-   在线局部重新规划；
-   真正动态避障返航。

V3 才属于真正意义上的实时避障返航。

当前阶段明确不做 V3。

------------------------------------------------------------------------

# 3. Manual 与 Mission 必须拆成两个独立接口

这是项目中的重要需求修正。

老板要求：

-   手动模式 Manual Return；
-   航点任务模式 Mission Return；

必须使用两个独立函数接口。

未来原则上为：

``` cpp
planManualReturn(...)
```

和：

``` cpp
planMissionReturn(...)
```

禁止为了"统一接口"提前写成：

``` cpp
planReturn(mode, ...)
```

当前只开发：

``` cpp
planManualReturn(...)
```

Mission 模式暂不开发。

原因：

### Manual 模式

主要输入：

``` text
实际历史轨迹
+
飞行过程中扫描到的局部 PCD
```

手动模式的 PCD 一般只覆盖飞机已经飞过附近区域，因此：

> PCD 中"没有点"不能自动认为"这里是自由空间"。

这是 Manual Return 设计的核心约束之一。

### Mission 模式

未来还会额外拥有：

``` text
预设 Mission 航点数组
+
通常更完整的全局 PCD
+
实际飞行轨迹
```

未来可以在 Manual Return 的基础上进一步利用 Mission 信息优化。

------------------------------------------------------------------------

# 4. 当前仿真工程

当前开发环境是一个 ROS1 无人机仿真项目。

主要组成包括：

-   EGO_Planner 相关规划工程；
-   港科大无人机仿真环境；
-   成熟无人机模型；
-   成熟运动控制模块；
-   激光雷达/地图模块；
-   RViz；
-   现有轨迹服务器和 Cascade PID 控制器。

当前原始仿真启动命令为：

``` bash
roslaunch test_interface single_drone_mid360.launch
```

可以：

``` text
启动仿真
→ RViz手动设置目标点
→ 无人机自动规划并飞向目标
```

现有局部规划系统具有动态避障能力。

但是：

> Manual Return V1 不允许依赖 EGO/diff_planner
> 的局部动态避障重新规划返航路径。

尤其禁止将 Home 重新作为 `/goal` 发布给 EGO/diff_planner 来"实现返航"。

那会变成：

``` text
trigger
→ Home作为新Goal
→ EGO重新规划
```

这不是本项目要求的"历史轨迹原路回溯"。

------------------------------------------------------------------------

# 5. 当前已经确认的 ROS 数据链路

Codex 第一阶段工程调查确认：

## 5.1 定位

``` text
Topic:
/quad_0/lidar_slam/odom

Type:
nav_msgs/Odometry

Frame:
world
```

Manual Return Recorder 从该 topic 获取真实位置。

## 5.2 RViz 目标

``` text
/goal
geometry_msgs/PoseStamped
```

进入现有规划器。

## 5.3 规划器

当前工程实际节点名称/体系为：

``` text
diff_planner
```

规划输出：

``` text
/quad0_planning/trajectory
traj_utils/PolyTraj
```

## 5.4 Trajectory Server

``` text
quad0_traj_server
```

将规划轨迹转换为控制器参考命令。

输出：

``` text
/quad_0/planning/pos_cmd
mars_quadrotor_msgs/PositionCommand
```

## 5.5 控制器

``` text
quad0_cascadePID_node
```

订阅：

``` text
/quad_0/planning/pos_cmd
```

输出：

``` text
/quad_0/cmdRPM
```

最终驱动仿真无人机。

## 5.6 地图

``` text
/map_generator/global_cloud
sensor_msgs/PointCloud2
```

当前 Manual Return V1.0 中，PCD/点云只用于：

``` text
加载
+
显示
+
为 Safe-RDP 预留
```

暂未用于安全碰撞判定。

------------------------------------------------------------------------

# 6. 当前仿真主要运动参数

现有工程调查得到：

``` text
max_vel = 1.5 m/s
max_acc = 6.0 m/s²
```

当前 Manual Return 默认：

``` text
max_reasonable_speed = 2.25 m/s
```

其来源：

``` text
1.5 × 现有 max_vel
```

用作轨迹异常速度告警阈值。

当前返航默认速度：

``` text
return_cruise_speed = 0.5 m/s
```

返航最大加速度初值：

``` text
max_return_acceleration = 1.5 m/s²
```

这些都属于仿真初始值，实际产品部署前需要重新标定。

------------------------------------------------------------------------

# 7. 当前无人机模型尺寸

Codex 调查到当前仿真模型只有 mesh 几何信息：

``` text
odom_visualization/meshes/yunque.dae
```

变换后的近似包围尺寸：

``` text
0.525 × 0.525 × 0.242 m
```

当前为未来 Safe-RDP 预留的保守球形包络半径：

``` text
约 0.39 m
```

注意：

> V1.0 中该半径尚未用于实际路径安全判定。

------------------------------------------------------------------------

# 8. 历史轨迹输入格式

当前测试轨迹采用 CSV。

固定字段：

``` text
timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw
```

定义：

``` text
timestamp : s
x/y/z     : m
vx/vy/vz  : m/s
roll      : rad
pitch     : rad
yaw       : rad
```

CSV 行顺序就是历史飞行时间顺序。

禁止为了"修复数据"自行按 timestamp 重新排序。

------------------------------------------------------------------------

# 9. 当前样例轨迹

已测试样例：

``` text
点数：393
时长：约39.2 s
采样周期中位数：约0.1 s
采样频率：约10 Hz
轨迹长度：约43.59 m
```

轨迹大致范围：

``` text
x : 0 ～ 41.755 m
y : -7.304 ～ 0 m
z : 约0.45 ～ 1.15 m
```

------------------------------------------------------------------------

# 10. 当前 Manual Return V1.0 软件包

所有返航相关代码必须位于：

``` text
src/manual_return_planner/
```

原则：

> 不修改旧工程源码。

当前包目录包括：

``` text
src/manual_return_planner/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
├── launch/
├── config/
├── test/
└── README.md
```

后续 AI 在修改代码前必须首先检查：

``` text
src/manual_return_planner/
```

和：

``` text
src/manual_return_planner/README.md
```

不要直接去修改
`test_interface`、`diff_planner`、`traj_server`、`cascadePID` 等旧模块。

------------------------------------------------------------------------

# 11. 当前 Manual Return 主接口

当前只允许存在：

``` cpp
planManualReturn(...)
```

其职责：

``` text
历史轨迹
→ 预处理
→ 反向
→ 3D RDP
→ 最大线段约束
→ yaw
→ ReturnPlanResult
```

Mission Return 暂时不要实现。

------------------------------------------------------------------------

# 12. V1.0 当前算法流程

``` text
10Hz历史轨迹
    ↓
合法性检查
    ↓
NaN / Inf检查
    ↓
timestamp检查
    ↓
近重复点过滤
    ↓
位置跳变检测/告警
    ↓
Reverse
    ↓
3D RDP
    ↓
最大线段长度约束
    ↓
返航航向yaw生成
    ↓
输出返航航点
```

------------------------------------------------------------------------

# 13. 当前关键算法参数

  参数                          当前仿真初值 含义
  --------------------------- -------------- ------------------------------
  `record_frequency`                 10.0 Hz 历史轨迹记录频率
  `min_point_spacing`                 0.03 m 近重复点过滤阈值
  `rdp_epsilon`                       0.05 m 3D RDP 最大允许偏差
  `max_segment_length`                 5.0 m 最大返航关键线段
  `max_reasonable_speed`            2.25 m/s 历史轨迹异常速度阈值
  `return_cruise_speed`              0.5 m/s 当前仿真返航巡航速度
  `max_return_acceleration`         1.5 m/s² 当前返航加速度初值
  `home_position_tolerance`           0.30 m Home 位置容差
  `control_takeover_delay`             1.5 s 当前代码中的控制接管等待参数
  `vehicle_body_radius`             约0.39 m Safe-RDP 预留值，V1未启用

以上参数均不是最终实机参数。

------------------------------------------------------------------------

# 14. 当前 RDP 样例测试结果

393 点样例：

``` text
raw points          = 393
preprocessed points = 316
RDP points          = 22
```

长度：

``` text
raw length          = 43.591 m
preprocessed length = 43.528 m
RDP length          = 43.503 m
```

最大 RDP 偏差：

``` text
0.0472 m
```

最大返航线段：

``` text
4.975 m
```

说明当前：

``` text
Reverse + 3D RDP
```

算法层表现符合预期。

------------------------------------------------------------------------

# 15. 为什么普通 RDP 还不是生产级方案

普通 RDP 只保证：

``` text
几何偏差小
```

但不能保证：

``` text
没有穿障碍
没有抄近路
没有离开已知区域
满足机体安全包络
```

所以后续必须升级为：

``` text
Safe-RDP
```

------------------------------------------------------------------------

# 16. Safe-RDP 总体定义

后续 V1.1 目标：

``` text
Safe-RDP
=
RDP
+
Historical Corridor
+
PCD Collision Check
+
Vehicle Safety Envelope
+
Shortcut Rejection
+
Path Validator
```

当前接口/参数已预留，但尚未正式启用。

------------------------------------------------------------------------

# 17. Historical Corridor

这是 Manual Return 最关键的安全约束之一。

手动模式地图一般只覆盖飞机已经飞过附近区域。

因此：

> "PCD 没有障碍点"不能解释成"这里一定自由"。

Safe-RDP 候选新线段必须始终位于原历史轨迹附近。

即建立：

``` text
历史轨迹安全管道
```

------------------------------------------------------------------------

# 18. Shortcut Rejection

V1 不允许明显"抄近路"。

原子轨迹长度：

``` text
L_history
```

新直线：

``` text
L_direct
```

定义：

``` text
rho = L_direct / L_history
```

如果 `rho` 很小，说明原轨迹可能是 U
型、绕柱、支路或折返，而候选线段正在直接切过去。

未来 Safe-RDP 会通过：

``` text
min_length_ratio
```

拒绝这种行为。

------------------------------------------------------------------------

# 19. PCD 在 Manual V1 中的正确定位

PCD 不用于：

> 主动探索新自由空间。

而主要用于：

> 否决危险的轨迹压缩。

未来流程：

``` text
PCD
 ↓
VoxelGrid
 ↓
KdTree
 ↓
候选返航线段采样
 ↓
最近障碍物距离
 ↓
clearance > R_safe ?
```

------------------------------------------------------------------------

# 20. 无人机安全半径

未来：

``` text
R_safe
=
R_body
+
E_localization
+
E_tracking
+
E_map
+
M_extra
```

其中：

``` text
R_body         = 机体包络
E_localization = 定位误差
E_tracking     = 跟踪误差
E_map          = 地图误差
M_extra        = 附加安全裕度
```

V1.0.1 需要通过实际仿真获取：

``` text
mean tracking error
P95 tracking error
max tracking error
```

用来确定未来 `E_tracking`。

------------------------------------------------------------------------

# 21. 当前 ROS 可视化与触发接口

当前 Manual Return 包使用：

``` text
/manual_return/raw_path
/manual_return/preprocessed_path
/manual_return/reversed_path
/manual_return/rdp_path
/manual_return/home
/manual_return/map_cloud
/manual_return/status
```

终端触发：

``` bash
rosservice call /manual_return/trigger "{}"
```

当前 V1.0 默认：

``` text
enable_return_command_output = false
```

因此当前版本默认只完成：

``` text
记录
规划
可视化
```

尚不能称为真实返航闭环。

------------------------------------------------------------------------

# 22. 当前最重要的未解决问题：控制权切换

现有控制链路：

``` text
diff_planner
    ↓
traj_server
    ↓
/quad_0/planning/pos_cmd
    ↓
cascadePID
```

Manual Return 如果直接向 `/quad_0/planning/pos_cmd` 发布命令，会存在两个
publisher 同时控制一个 controller 的风险。

这不允许。

所以 V1.0.1 必须增加：

``` text
CommandGate
```

------------------------------------------------------------------------

# 23. V1.0.1 目标控制拓扑

目标：

``` text
traj_server
    ↓
/manual_return/normal_pos_cmd
                  │
                  ▼
             CommandGate
                  │
                  ▼
/quad_0/planning/pos_cmd
                  │
                  ▼
             cascadePID

ManualReturnExecutor
    ↓
/manual_return/return_pos_cmd
                  │
                  └────→ CommandGate
```

要求：

``` text
/quad_0/planning/pos_cmd
```

始终只有 `CommandGate` 一个 publisher。

------------------------------------------------------------------------

# 24. `/mandatory_stop_to_planner`

当前 README 记录：触发返航时会发布：

``` text
/mandatory_stop_to_planner
```

但下一阶段必须确认：

1.  哪个 node subscribe；
2.  消息类型；
3.  是否只停止重新规划；
4.  是否停止 traj_server；
5.  traj_server 是否继续输出旧轨迹。

核心原则：

> `/mandatory_stop_to_planner` 可以作为辅助停止措施，但不能替代
> CommandGate。

------------------------------------------------------------------------

# 25. V1.0.1 返航起点锚定

历史轨迹以 10 Hz 记录，因此 trigger 时最后保存点最多可能滞后约 0～100
ms。

所以 trigger 时应记录：

``` text
current_pose_at_trigger
```

并将其强制作为：

``` text
return start anchor
```

要求：

``` text
第一返航参考点 ≈ trigger时真实位置
```

------------------------------------------------------------------------

# 26. V1.0.1 返航执行

RDP 生成的关键点：

``` text
P0 → P1 → P2 → ...
```

不能直接离散跳点发送给底层控制器。

返航执行器必须生成连续参考轨迹。

V1.0.1 允许使用：

``` text
piecewise linear trajectory
```

暂时不要求 B-Spline。

------------------------------------------------------------------------

# 27. V1.0.1 返航状态机

建议：

``` text
RECORDING
    ↓
PLANNING
    ↓
WAITING_FOR_TAKEOVER
    ↓
RETURNING
    ↓
FINISHED
```

异常：

``` text
FAILED
```

返航开始以后，返航轨迹不能再追加进 outbound 历史轨迹。

但可以单独保存：

``` text
executed_return_path.csv
```

用于后续误差分析。

------------------------------------------------------------------------

# 28. 返航结束条件

建议同时满足：

``` text
distance_to_home < home_position_tolerance
```

和：

``` text
current_speed < return_finish_speed_threshold
```

当前初值建议：

``` text
home_position_tolerance = 0.30 m
return_finish_speed_threshold = 0.15 m/s
```

到达 Home 后保持 MANUAL_RETURN 控制，不自动恢复 NORMAL。

------------------------------------------------------------------------

# 29. V1.0.1 必须记录跟踪误差

返航过程中记录：

``` text
reference position
actual position
```

定义：

``` text
e(t) = |p_actual - p_reference|
```

输出：

``` text
mean_tracking_error
P95_tracking_error
max_tracking_error
```

并保存：

``` text
return_tracking_log.csv
```

推荐字段：

``` text
timestamp,
ref_x,ref_y,ref_z,
actual_x,actual_y,actual_z,
error_x,error_y,error_z,
error_norm
```

这些数据将直接用于 Safe-RDP 的 `tracking_margin` 定参。

------------------------------------------------------------------------

# 30. V1.0.1 完整验收流程

启动：

``` bash
roslaunch manual_return_planner manual_return_mid360.launch
```

然后：

``` text
RViz设置目标
    ↓
无人机正常飞行
    ↓
Manual Return后台10Hz记录
    ↓
终端trigger
```

触发：

``` bash
rosservice call /manual_return/trigger "{}"
```

然后：

``` text
冻结历史
→ Reverse
→ RDP
→ CommandGate切换
→ Manual Return执行
→ 无人机沿返航路线返回
→ Home
```

------------------------------------------------------------------------

# 31. V1.0.1 唯一控制源验收

运行：

``` bash
rostopic info /quad_0/planning/pos_cmd
```

必须确认：

``` text
Publishers:
仅 CommandGate
```

正常飞行和返航阶段都一样。

这是 V1.0.1 的核心验收指标之一。

------------------------------------------------------------------------

# 32. 完整仿真当前环境阻塞

当前完整 workspace 在现有环境中受到：

``` text
mars_local_sensing
```

缺少：

``` text
glfw3
```

开发依赖影响。

已知：

-   `manual_return_planner` 独立包编译通过；
-   7 个 GTest 全部通过；
-   ROS Noetic WSL 离线节点验证通过；
-   完整 workspace 尚未完成真实运行闭环。

下一阶段需要先解决依赖，然后真实启动 `manual_return_mid360.launch`。

------------------------------------------------------------------------

# 33. 当前测试状态

## 已完成

``` text
Manual Return 独立 ROS package
CSV I/O
10Hz Recorder
NaN/Inf validation
timestamp validation
near-duplicate filtering
position jump warning
Reverse
3D RDP
max segment length
yaw generation
RViz visualization
PCD visualization
trigger service
status
离线CSV验证
7个GTest
```

## 已验证样例

``` text
393 → 316 → 22 points
RDP max deviation = 0.0472 m
RDP path length = 43.503 m
```

## 尚未完成

``` text
CommandGate
单控制源
完整仿真启动
实际返航飞行
返航跟踪误差统计
Safe-RDP
PCD碰撞安全判定
Historical Corridor
Shortcut Rejection
Mission Return
V2
V3
```

------------------------------------------------------------------------

# 34. 严格开发顺序

``` text
V1.0
算法与可视化
    ↓

V1.0.1
真正控制接管与实际返航
    ↓

V1.1
Safe-RDP
    ↓

V1.2
必要的平滑/轨迹执行体验优化
    ↓

V2
冗余空间路线优化
    ↓

V3
实时动态避障返航
```

不要跳阶段。

------------------------------------------------------------------------

# 35. V1.1 Safe-RDP 预定工作

V1.0.1 跑通以后才开始：

``` text
Historical Corridor
PCD KDTree
Vehicle Safety Envelope
Collision Check
Shortcut Ratio
Path Validator
```

对每一个 RDP 候选压缩段：

``` text
Pi → Pj
```

判断能否安全替换：

``` text
Pi → Pi+1 → ... → Pj
```

至少同时满足：

1.  3D RDP deviation \<= epsilon；
2.  候选线段位于历史轨迹 corridor 内；
3.  candidate length \<= max_segment_length；
4.  shortcut ratio 满足要求；
5.  PCD clearance \> R_safe。

------------------------------------------------------------------------

# 36. V1.2 平滑原则

如果后续加入 B-Spline 或圆角：

``` text
Safe-RDP polyline
    ↓
尝试 smooth
    ↓
重新做完整安全检查
```

如果平滑后不安全：

``` text
回退 Safe-RDP polyline
```

禁止为了"看起来顺滑"牺牲安全。

------------------------------------------------------------------------

# 37. V2 的路线优化边界

V2 才允许真正删除：

``` text
拍照支路往返
长时间停留区域
重复绕行
其他非必要空间路线
```

因此 V1 测试中"支路拍照后回主线"必须保留。

这是 V1 和 V2 的边界测试。

------------------------------------------------------------------------

# 38. V3 实时动态避障

V3 才会考虑：

``` text
实时 odom
实时 local map
动态障碍
局部 planner
在线 replan
```

当前任何 AI 都不要提前将 EGO 的实时避障重新引入 V1/V1.1。

------------------------------------------------------------------------

# 39. 当前测试输出文件

当前算法可以输出：

``` text
raw_path.csv
preprocessed_path.csv
reversed_path.csv
rdp_return_path.csv
```

V1.0.1 还应增加：

``` text
executed_return_path.csv
return_tracking_log.csv
```

------------------------------------------------------------------------

# 40. 代码隔离原则

这是长期必须遵守的硬性规则：

> 所有返航相关新代码优先只放在 `src/manual_return_planner/`。

禁止未经人工明确批准修改：

``` text
test_interface
diff_planner
traj_server
cascadePID
simulator
mars_local_sensing
现有旧launch
现有控制器
```

如果发现真的必须修改旧工程：

1.  不要先改；
2.  说明为什么无法通过 remap / gate / wrapper 解决；
3.  给出最小修改方案；
4.  等人工确认。

------------------------------------------------------------------------

# 41. AI 接手项目时的强制阅读顺序

任何新的 AI / Codex 会话接手时，应先阅读：

``` text
1. 本文档
2. src/manual_return_planner/README.md
3. src/manual_return_planner/launch/
4. src/manual_return_planner/config/
5. include/manual_return_planner/
6. src/manual_return_planner/src/
7. test/
```

然后再检查：

``` text
test_interface/launch/single_drone_mid360.launch
```

以及实际 ROS graph。

禁止只看一个 cpp 就直接开始重构。

------------------------------------------------------------------------

# 42. AI 接手后的第一条确认

新的 AI 应首先回答：

``` text
当前代码版本是什么？
当前阶段做到哪里？
下一阶段目标是什么？
有哪些功能明确禁止提前做？
当前控制链路是什么？
```

如果回答不出这些问题，说明其尚未充分理解项目。

------------------------------------------------------------------------

# 43. AI 不得错误理解的关键点

## 43.1 不是普通 Return To Home

而是：

``` text
History-based Backtracking Return
```

## 43.2 不能把 Home 发给 EGO planner 来冒充原路返航

## 43.3 Manual PCD 未观测区域是"未知"，不是"自由空间"

## 43.4 V1 删除的是采样冗余，不是实际空间路线

## 43.5 Safe-RDP 用于否决危险压缩，不用于主动寻找捷径

## 43.6 Mission 模式未来使用独立接口，不提前混入 Manual Return

------------------------------------------------------------------------

# 44. 故障与回退理念

返航属于安全功能。

后续最终应形成：

``` text
优先：
Safe smooth path

失败：
Safe-RDP polyline

再失败：
更密集历史反向轨迹

仍不安全：
NO_SAFE_PATH
```

绝不能为了"保证一定返航"不断降低安全距离。

如果 PCD 明确显示历史路线已经被堵，应返回：

``` text
NO_SAFE_PATH
```

交给上层执行悬停、降落或人工接管。

------------------------------------------------------------------------

# 45. 项目长期原则

整个项目后续开发要坚持：

``` text
先闭环
再安全
再平滑
再优化
再实时
```

即：

``` text
V1.0     算法可运行
V1.0.1   控制闭环
V1.1     路径安全
V1.2     控制/平滑优化
V2       路线效率
V3       动态避障
```

不要将五个问题同时塞进一个版本。

------------------------------------------------------------------------

# 46. 当前下一步任务

当前最优先任务不是 Safe-RDP。

当前必须先完成：

> **Manual Return V1.0.1：CommandGate + 完整仿真闭环返航。**

具体目标：

``` text
启动 manual_return_mid360.launch
    ↓
RViz 设置目标
    ↓
无人机正常飞行
    ↓
10Hz记录
    ↓
终端 trigger
    ↓
冻结历史
    ↓
Reverse + RDP
    ↓
切换 CommandGate
    ↓
Manual Return 执行
    ↓
返回 Home
```

并记录：

``` text
trigger start error
mean tracking error
P95 tracking error
max tracking error
final home error
```

------------------------------------------------------------------------

# 47. V1.0.1 结束后的下一步

只有真正满足：

``` text
实际无人机在仿真中沿历史RDP路径返回Home
```

才进入：

``` text
Manual Return V1.1 Safe-RDP
```

V1.1 参数将结合：

``` text
无人机包络
定位误差
实际tracking error
PCD点云密度
当前RDP效果
```

重新定标。

------------------------------------------------------------------------

# 48. 交接时建议同时提供的材料

如果以后转交新的 AI，建议同时发送：

``` text
本项目交接说明.md
src/manual_return_planner/README.md
src/manual_return_planner 整个文件夹
当前最新终端报错
当前RViz截图/录屏
当前最新测试CSV
相关PCD
```

如果是 V1.0.1 以后，还应提供：

``` text
return_tracking_log.csv
executed_return_path.csv
```

------------------------------------------------------------------------

# 49. 新 AI 的推荐工作方式

新的 AI 应采用：

``` text
Read
→ Inspect
→ Understand
→ Compile
→ Test
→ Modify
→ Re-test
```

不要采用：

``` text
Guess
→ Rewrite
```

特别是 ROS topic、frame、控制器接口和 launch 拓扑：

> 必须以当前实际工程为准，不允许凭经验猜测。

------------------------------------------------------------------------

# 50. 新 AI 接手检查清单

新的 AI 接手时，应逐项确认：

-   [ ] 已完整阅读本交接文档；
-   [ ] 已完整阅读 `src/manual_return_planner/README.md`；
-   [ ] 已确认当前版本仍为 Manual Return，Mission Return 尚未开发；
-   [ ] 已确认当前 V1.0 只完成算法/可视化，不等于真实返航闭环；
-   [ ] 已确认下一步是 V1.0.1 CommandGate，而不是 Safe-RDP；
-   [ ] 已确认定位 topic 为 `/quad_0/lidar_slam/odom`；
-   [ ] 已确认 world frame 为 `world`；
-   [ ] 已确认规划输出为 `/quad0_planning/trajectory`；
-   [ ] 已确认控制器输入为 `/quad_0/planning/pos_cmd`；
-   [ ] 已确认地图 topic 为 `/map_generator/global_cloud`；
-   [ ] 已确认现有 V1.0 样例结果为 393→316→22 点；
-   [ ] 已确认 `rdp_epsilon=0.05 m` 只是仿真初值；
-   [ ] 已确认不能把 Home 作为 EGO goal 来实现返航；
-   [ ] 已确认 Manual PCD 中"没有点"不能当作自由空间；
-   [ ] 已确认旧源码不能在未经人工允许的情况下修改；
-   [ ] 已确认 `/mandatory_stop_to_planner` 不能默认等价于安全控制接管；
-   [ ] 已确认 `/quad_0/planning/pos_cmd` 未来必须只有一个有效
    publisher；
-   [ ] 已确认 V1.0.1 完成后才进入 Safe-RDP；
-   [ ] 已确认 V2 才允许删除真实空间冗余路线；
-   [ ] 已确认 V3 才引入实时动态避障。

------------------------------------------------------------------------

# 51. 一句话交接说明

> 这是一个基于 ROS1
> 无人机仿真环境开发的"历史实际飞行轨迹原路返航"项目；Manual Return V1.0
> 已完成 10Hz 轨迹记录、预处理、Reverse 和 3D RDP
> 压缩验证，当前下一步不是做 Safe-RDP，而是通过独立 CommandGate 实现
> `/quad_0/planning/pos_cmd` 的单控制源切换，让无人机在 RViz
> 正常飞行后能通过 `/manual_return/trigger` 真正沿反向 RDP 路径返回
> Home，随后再根据实际跟踪误差进入 Safe-RDP 安全约束开发。

------------------------------------------------------------------------

# 52. V1.0.1\~V1.0.3.1 后续阶段完成情况（新增）

## 52.1 V1.0.1：单控制源接管与真实返航闭环

在原 V1.0 基础上，已完成：

-   CommandGate 控制权切换机制；
-   Manual Return Executor；
-   `/quad_0/planning/pos_cmd` 单一 publisher 控制链路；
-   trigger 后真实返航执行；
-   返航状态机。

当前控制拓扑：

``` text
traj_server
    ↓
/manual_return/normal_pos_cmd
              \
               \
                → CommandGate → /quad_0/planning/pos_cmd → cascadePID
               /
ManualReturnExecutor
    ↓
/manual_return/return_pos_cmd
```

已验证：

``` text
RViz设置目标
    ↓
无人机正常飞行
    ↓
10Hz记录历史轨迹
    ↓
trigger返航
    ↓
Reverse + RDP
    ↓
CommandGate切换
    ↓
无人机沿返航路径返回Home
```

------------------------------------------------------------------------

# 53. V1.0.2：统一评价指标体系

为了比较不同返航算法，建立统一 `return_metrics.csv` 输出体系。

当前指标包括：

## 输入规模

-   original_points
-   original_length_m

## 压缩效果

-   simplified_points
-   point_reduction_percent
-   return_length_m
-   length_change_percent

## 路径保持

-   max_deviation_m
-   mean_deviation_m
-   p95_deviation_m

## 安全

-   min_clearance_m
-   unsafe_segments
-   validated_segments
-   collision_check_count

## 跟踪

-   cross_track_p95
-   cross_track_max
-   along_track_p95
-   vertical_error_p95

## 完成质量

-   final_home_error_m
-   return_duration_s

## 性能

-   planning_time_ms
-   memory_usage_mb
-   pointcloud_size
-   voxelized_cloud_size

这些指标与算法实现解耦，未来 Safe-RDP、shortcut optimization
等算法必须保持兼容。

------------------------------------------------------------------------

# 54. V1.0.3：Benchmark实验体系建立

新增：

-   scenario场景标签；
-   RViz Benchmark可视化；
-   多运行结果保存；
-   自动汇总分析脚本。

新增RViz显示：

``` text
/manual_return/history_path
/manual_return/return_path_line
/manual_return/key_points
/manual_return/home_marker
/manual_return/current_position
```

显示含义：

-   蓝色：无人机历史飞行轨迹（返航算法输入）
-   红色：RDP压缩后的返航路径
-   红色球：关键返航航点
-   绿色球：当前无人机位置
-   Home标记：返航终点

------------------------------------------------------------------------

# 55. V1.0.3.1：Benchmark数据修正与最终标定结果

完成：

-   scenario字段修复；
-   tracking汇总脚本修复；
-   benchmark完整性检查；
-   多场景实验统计。

测试场景：

  场景             说明
  ---------------- ----------
  straight_long    长直线
  straight_short   短直线
  turn90           90度转弯
  s_curve          S型轨迹
  height_change    高度变化

全部5组实验通过：

``` text
runs: 5
ok: 5
fail: 0
```

------------------------------------------------------------------------

# 56. V1.0.3.1 Benchmark关键结果

## 56.1 RDP压缩效果

平均压缩率：

约：

97.8%

说明：

10Hz历史轨迹存在大量采样冗余，RDP能够有效降低关键点数量。

## 56.2 路径保持

五个场景：

最大RDP偏差：

约：

0.048m

说明：

当前：

``` text
Reverse + 3D RDP
```

能够较好保持原始飞行走廊。

## 56.3 Tracking误差标定

多场景测试：

cross-track最大约：

0.138m

因此当前仿真环境推荐：

``` text
tracking_margin = 0.15m
```

该参数作为 V1.1 Safe-RDP 初始值。

注意：

该值来自当前仿真模型，需要后续结合真实无人机定位误差重新标定。

------------------------------------------------------------------------

# 57. 当前项目正式进入 V1.1 Safe-RDP阶段

当前状态：

``` text
V1.0
历史轨迹Reverse + RDP
        ↓
V1.0.1
真实闭环返航
        ↓
V1.0.2
指标体系
        ↓
V1.0.3
Benchmark体系
        ↓
V1.0.3.1
多场景标定完成
        ↓
V1.1 Safe-RDP
```

------------------------------------------------------------------------

# 58. V1.1 Safe-RDP目标

V1.1不是重新规划。

不是实时避障。

目标：

> 在历史返航路径压缩过程中，引入地图和安全约束，保证压缩后的路径仍然安全。

总体流程：

``` text
历史轨迹
    ↓
Reverse
    ↓
RDP候选路径
    ↓
历史轨迹走廊约束
    ↓
PCD碰撞检测
    ↓
无人机安全包络检查
    ↓
Safe-RDP路径
```

------------------------------------------------------------------------

# 59. V1.1新增安全模型

安全距离：

``` text
R_safe
=
R_body
+
E_tracking
+
E_localization
+
E_map
+
M_extra
```

当前初始参数：

  参数                      初值
  ----------------- ------------
  tracking_margin          0.15m
  vehicle_radius         约0.39m
  extra_margin        待实验确定

注意：

V1.1第一阶段不追求极限压缩，而优先保证安全。

------------------------------------------------------------------------

# 60. V1.1开发边界

第一阶段只实现：

-   PCD加载；
-   点云降采样；
-   KDTree查询；
-   路径安全检测；
-   clearance计算；
-   unsafe segment识别。

暂不实现：

-   shortcut优化；
-   主动绕障；
-   新路径搜索；
-   实时动态避障。

------------------------------------------------------------------------

# 61. V1.1验收目标

完成后应能够回答：

1.  RDP路径是否经过障碍物？
2.  当前安全距离是多少？
3.  哪些segment不安全？
4.  Safe-RDP是否拒绝危险压缩？
5.  不同场景下安全指标如何变化？

------------------------------------------------------------------------

# 62. 当前AI接手后的最新任务

新的AI接手后，应优先执行：

1.  阅读本交接文档；
2.  阅读 `src/manual_return_planner/README.md`；
3.  确认当前版本为 V1.0.3.1；
4.  不修改旧规划器和控制器；
5.  开始 V1.1 Safe-RDP 基础版开发。

当前禁止：

-   提前开发Mission Return；
-   提前接入EGO动态避障；
-   修改diff_planner；
-   修改cascadePID；
-   将Home重新作为planner goal。

------------------------------------------------------------------------

# 63. 当前一句话交接

> Manual Return项目已经完成从历史轨迹采集、Reverse+3D
> RDP压缩、ROS闭环返航、Benchmark评价体系建立到多场景误差标定的完整V1阶段验证，目前进入V1.1
> Safe-RDP阶段，需要在不改变历史轨迹返航逻辑的基础上，引入PCD安全验证、无人机安全包络和历史轨迹走廊约束，实现安全型返航路径压缩。
