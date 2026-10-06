# 智能车：分层复现版

按照 **驱动 → 建图 → 定位 → 单点导航 → 多点导航** 逐层运行与验收。包含少换向优先搜索、可行路线保留、分阶段低代价线导航和巡检任务规划。

先读 [逐步复现手册](docs/REPRODUCE.md)，接实车时对照 [驱动接口](docs/DRIVER_CONTRACT.md)，调整参数看 [导航调参说明书](docs/NAVIGATION_TUNING.md)和[车辆参数参考](docs/PARAMETERS.md)。目录与依赖见 [架构说明](docs/ARCHITECTURE.md)。

```text
src/
  smartcar_drivers/       实车驱动接入入口、分层检查
  smartcar_description/   仿真车体模型
  smartcar_sim/           Gazebo 底盘插件、模拟雷达与仿真工具
  smartcar_mapping/       建图
  smartcar_localization/  激光里程计与全地图/选墙匹配，保留 AMCL
  smartcar_navigation/    单点全局规划、局部跟踪、导航参数
  smartcar_mission/       多点任务顺序管理
  smartcar_bringup/       各层组合入口、RViz 配置
  smartcar_region_tool/   RViz 低代价线工具
config/                  可编辑配置，仿真与实车导航配置分开
scripts/                 robot.sh 本机入口、sim.sh Docker 入口
data/                    地图和运行数据
tests/                   离线测试及需 ROS 的测试
docs/                    当前架构与复现说明
archive/                 停用代码，不参与构建
legacy-reference.zip     旧版备份与参考材料，不参与构建
```

正式源码直接位于 `src/`，不再通过“解压工作空间 + docker/overlay 覆盖”构建。实车编译可以排除 Gazebo 包。厂商底盘与雷达协议未知，因此提供接入规范和启动挂接点，未宣称已经完成实车驱动。

交互测试 C 任务计划与原 nav-test：保持仿真定位运行，退出普通导航后执行以下命令，按提示输入 A/B 配置、画低代价线，再按回车测试。编号固定从右到左（右侧 1/2，左侧 9/10）。

```bash
bash ~/smartcar1/scripts/sim.sh plan-test
```

## 仿真快速启动

16 点 C 巡检计划已接入现有多点队列：只需标定 10 个巡检点、左右各 2 个绕行点、起点和终点，方向由程序生成。标定、预览、加载及执行命令见 [16 点巡检计划](docs/INSPECTION_PLAN.md)。

默认使用窄场地、雷达帧间里程计＋全地图墙面匹配定位，以及现有 Hybrid A* 导航方案。分层启动，各长期运行命令使用独立终端。

**终端 1：启动仿真和定位，保持运行。**

```bash
bash ~/smartcar1/scripts/sim.sh start
```

**终端 2：打开 RViz，用 2D Pose Estimate 设置初始位姿。**

```bash
bash ~/smartcar1/scripts/sim.sh rviz
```

**终端 3：启动导航，使用 2D Nav Goal 下发目标。**

```bash
bash ~/smartcar1/scripts/sim.sh nav
```

`nav` 只启动导航，自动识别当前场地；每次启动时低代价线为空，需要重新绘制。RViz 的 **Draw Low-Cost Lines**（快捷键 **L**）可以继续画线，每条完成后自动保存。退出导航不会停止仿真、定位或 RViz。

**切换键盘控制：在导航终端按 Ctrl+C，等待退出，再运行：**

```bash
bash ~/smartcar1/scripts/sim.sh keyboard-sim
```

结束键盘控制后按 Ctrl+C，再运行 `nav` 即可恢复导航，不需要重设初始位姿。修改 `config/navigation.yaml` 后同样只需退出并重启 `nav`，不需要 build。

特殊横移“向左/右寻找 1.5 m 点”的距离，可在 `config/navigation.yaml` 的 `tuning:` 下修改：

```yaml
tuning:
  line_entry_distance: 1.5    # 第一段沿低代价线向左/右偏移的距离，单位米
  line_retreat_distance: 1.5  # 第二段相对目标投影点的退让距离，单位米
```

这两项独立生效，适用于两种特殊横移。只想调整左右寻找距离，就改已有的 `line_entry_distance`，例如改成 `1.2`；不要重复添加 `tuning:`。第一段以当前后轴中心在线段上的最近点为起点，沿线偏移；方向由动作类型和目标位置决定。距离必须大于 0，改小不保证一定可达，规划仍检查车身碰撞和转弯约束。

默认 `start` 选择窄场地；切换场地使用以下命令之一。先退出原定位、导航、键盘终端。切换会重建仿真并重置车辆，需重新设置初始位姿：

```bash
bash ~/smartcar1/scripts/sim.sh start normal
bash ~/smartcar1/scripts/sim.sh start narrow
```

`start` 自动配套 Gazebo 场景和定位地图；匹配的已有定位会复用（仍由原终端管理）。`nav` 不启动或重建仿真，也不启动定位；未运行定位时会提示先执行 `start`。

| 场地 | 定位地图 | 低代价线（宿主机路径，相对项目目录） |
| --- | --- | --- |
| `normal` | `data/maps/sim_field_v1/map.yaml` | `data/navigation/low_cost_lines.json`（保留原有记录） |
| `narrow` | `data/maps/sim_field_narrow/map.yaml` | `data/maps/sim_field_narrow/low_cost_lines.json` |

`start` 终端的 Ctrl+C 只停止其启动的定位；`nav` 终端的 Ctrl+C 只停止导航。停止整个仿真：

```bash
bash ~/smartcar1/scripts/sim.sh stop
```

首次运行或修改 ROS 源码后先构建镜像；本次快捷命令和 README 的修改不需要重新 build：

```bash
bash ~/smartcar1/scripts/sim.sh build
```

默认定位使用雷达帧间里程计＋全地图墙面局部匹配，不需要画框、选墙或轮脉冲。用 `bash ~/smartcar1/scripts/sim.sh wall-status` 查看 `target_mode: full_map` 与 `state: MATCHED`。当前不会自动按通道内外切换算法。

分层调试仍保留 `drivers`、`localization`、`single` 等入口。仅建图或单独调试驱动时使用 `bash ~/smartcar1/scripts/sim.sh drivers`（原场地，仅启动驱动，不启动定位）。手动选墙、AMCL、建图和实车流程见 [逐步复现手册](docs/REPRODUCE.md)。首次使用分层版应先停止旧版容器，不能让两套 ROS 共用端口同时运行。

实车当前仍针对 ROS1 Kinetic 接口；没有执行 ROS 版本升级。Linux/ROS/Gazebo 的构建和实际运行需要在目标环境验收，离线检查结果见 `docs/VALIDATION.md`。

通道内外判断：复用四角画框工具，按车辆定位参考点是否进框发布状态，操作见 [通道判断说明](docs/CORRIDOR.md)。

手动墙体特征定位：分别画线选择通道内/外优先墙，将线膨胀成区域，匹配区域覆盖的地图墙体，支持编号删除及保存加载，见 [墙体定位说明](docs/WALL_FEATURES.md)。

轮距融合仿真：模拟有符号累计脉冲，与墙体定位融合，并同步对比纯雷达估计，见 [轮距融合验证](docs/WHEEL_FUSION.md)。

低代价线默认不自动恢复（`load_saved_low_cost_lines: false`）：在 RViz 点击 **Draw Low-Cost Lines**（快捷键 **L**），依次点击起点和终点；每条完成后仍自动保存，但重启导航或切换地图时不读取旧线，请重新绘制。`nav` 使用上表中对应场地的文件；单独启动 `single` 或 `multi` 默认使用 `data/navigation/low_cost_lines.json`，重启容器也保留。新画的线用于下一次规划，不会突然修改正在执行的路径。清空并保存用：

```bash
bash ~/smartcar1/scripts/sim.sh lines-clear
```

墙体居中修正仅在 `STRAIGHT` 模式且当前路径段为直线时参与，修正后的转向仍经过转向速率限制、轨迹预测及碰撞检查。普通模式和弯道不再被墙体居中修改。

特殊横移只选择最终目标前轴中点到线段距离最近的低代价线（等距按绘制顺序）。没有低代价线时，两种横移均使用普通导航。有线时，不尝试另一条线；正常导航遇到失败会保留任务，按下述恢复流程重试。第一段成功后锁定线的坐标及阶段目标，重规划不重新选线，第二段也使用锁定的线。

两种横移均不限制车身纵向位移：横向距离达到 0.20 m 后，按现有航向条件分类；60°～120° 的转向分类仍优先，朝向差达到 150° 为横移掉头，其余为普通横移。

巡检深度限制：标定点自动生成 x/y 各 ±0.4 m 的区域。前往任意 A/B 巡检点都不得越过进入方向的远端深度边界；从 B 区域发起下一目标时保留原深度边界，规划器自行寻找退出路径，不增加退出点。正式导航与 plan-test 共用全局约束。16 点坐标无需重标，重启任务程序重新生成计划即可。

### 正常导航的持续恢复

正常 `nav` 在雷达超时、控制超时、路径受阻、跟踪偏离或规划失败时停车并保留任务，不限制恢复重规划次数。雷达、定位、控制周期恢复，连续获得新扫描且确认车轮停稳后，间隔至少 2 秒继续尝试。`tuning.max_replans` 是旧测试恢复分支的参数，不限制正常导航的持续恢复。

恢复规划允许前轴中点落在**原始目标半径 8 cm 的圆内（停车按10 cm验收）**，朝向和巡检深度限制仍生效，目标圆心不会随重试漂移。中间的低代价线阶段仍按原阶段目标执行。若区域规划（或当前必要的线阶段）在本轮预算内没找到路径，则尝试返回最近一个与当前位置明显不同、已确认停稳的位置，包括上一个任务停车点、换向点和低代价线停车点；返回后再规划原任务。返回也需要重新碰撞检查，不会直接倒放历史路线。返回失败则停车等待并持续重试返回；没有可用历史停车点时，留在当前位置重试原任务。

日志 `RECOVERY_WAIT` 表示保留任务等待恢复，`RECOVERY_RETURN` 表示准备返回停车点，`RECOVERY_RETURNED` 表示已返回并准备再次规划。搜索超时不等于证明区域绝对不可达。取消任务、替换目标及正常完成仍按原流程处理；地图切换需重新下发目标。停车历史只在本次导航进程及当前地图内有效。`plan-test/nav-test` 保持有限测试并报告失败，不进入无限恢复。

取消持续恢复：

```bash
bash ~/smartcar1/scripts/sim.sh multi-cancel
```

停车确认使用连续 odom 位姿窗口与实际轮速；地图匹配修正不参与判定车辆是否静止。目标到达、碰撞及巡检边界仍使用当前地图定位。每次恢复规划完成后都会重新开始停车确认计时，包括已经位于目标区域内的零长度路径。

同一轮恢复会固定返回停车点，避免两个历史点之间反复往返；新的正常停车点或新任务会解除该固定。仅遇到搜索预算耗尽时，后续搜索预算按原值的 2 倍、4 倍递增，上限为 20 秒（原配置大于 20 秒时保留原配置），不改变碰撞检查，也不限制重试次数。

导航执行效率优化：全局规划对换向、很短的方向段和转向突变增加软代价；转向未达到要求时限制行驶距离。低代价线仍锁定目标最近线，入口①仅从原标称点沿到达朝向的车尾方向偏移0.3、0.6、0.9米，依次搜索三个候选（共享20秒预算，不另试标称点），找到无需前进/倒车换向的一段连续路径就立即采用；否则在预算内优先换向次数少的路径，再比较预计行驶时间，允许绕远以减少调整，单独规划到入口①并确认停稳，再整体规划①→退让点②→最终目标。②同样从原标称退让点沿车尾方向额外偏移0.3、0.6、0.9米；两段各自无需换向且整体可行时立即采用；否则比较整条路线实际换向次数（包括②处），相同时比较预计时间；默认后两段搜索预算15秒，两段都成功才执行，②处不重新丢弃末段路径。搜索失败时保持停车并走原有重试机制。局部跟踪失败位置在当前目标内形成有限的软代价，不禁止必要通行，不改变碰撞或巡检深度约束。相关参数位于 navigation.yaml 的 tuning 中：planner_switch_cost、planner_steer_cost、planner_short_segment_cost、line_entry_search_span、line_entry_timeout、line_entry_steer_seconds、line_search_span、line_suffix_timeout、tracking_turn_allowance。

两个 search_span 参数默认均为0.9米，三个单向偏移分别为该值的1/3、2/3、1倍；不是直接将原1.5米标称距离替换为0.3米。

完整任务测试还记录 elapsed、direction_changes、recoveries、planning_attempts、max_tracking_error 和 max_true_path_error。基线日志未记录的指标标为缺失，不用零代替；比较时间须同时检查任务完成率和跟踪误差。预计路线时间不是实际完成时间的保证。


B1B2A3A4A5B6B7A8B9B10
A1A2B3B4A5B6A7A8A9A10
B1B2B3A4A5A6A7A8B9A10
A1A2A3B4B5A6B7A8A9A10
A1A2A3A4A5A6A7B8B9B10
A1A2A3B4B5A6A7A8B9B10
B1A2A3B4A5A6B7B8A9B10
A1B2B3B4B5B6B7A8B9A10
B1A2B3B4A5A6B7A8A9B10
B1A2A3B4B5B6A7A8A9B10

普通模式单次规划默认最多20秒，由 tuning.normal_planning_timeout 设置；先保留可行路径，再用剩余预算优化换向次数；无需换向时可提前返回。修改后重启导航生效。

恢复模式：候选终点限制在原目标参考点8厘米半径内，实际停车按10厘米半径验收，保留2厘米余量；朝向和碰撞约束仍有效。

少换向搜索：先保存完整可行路径，再用剩余预算尝试限制换向次数；超时或优化失败保留已成功路线，取消任务则不执行。整条路线实际换向次数优先，相同次数比较预计时间。找不到任何完整可行路线时仍报告失败，不能保证所有几何场景有解。
