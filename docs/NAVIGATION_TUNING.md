# 导航调参说明书

本版优先减少**前进与倒车之间的换向次数**，允许路径适当变长。少打方向、路程最短、执行最快是不同指标，不能互相替代。这里的“行驶段”指连续前进或连续倒车的一段，不是路径中的栅格点数量。

## 配置入口与生效

- 仿真：`config/navigation.yaml`。
- 实车模板：`config/navigation-real.yaml`，使用前核对真实几何、反馈接口和执行器模型。
- 车辆参考点/底盘对接：[PARAMETERS.md](PARAMETERS.md)。任务点标定：[INSPECTION_PLAN.md](INSPECTION_PLAN.md)。

在仓库目录运行校验：

```bash
cd ~/smartcar1
python3 scripts/check-navigation-config.py config/navigation.yaml
python3 scripts/check-navigation-config.py config/navigation-real.yaml
```

改 YAML 后停止并重新启动导航或 plan-test；不要在同一文件中增加第二个 `tuning:` 块。默认仿真容器挂载宿主机 config，改参数无需 build。修改源码或首次拉取本版本需要构建：

```bash
bash ~/smartcar1/scripts/sim.sh build
bash ~/smartcar1/scripts/sim.sh start normal
```

`start normal` 启动大图仿真和定位，可能重建容器、重置位姿。之后分别在其他终端运行：

```bash
bash ~/smartcar1/scripts/sim.sh rviz-multi
bash ~/smartcar1/scripts/sim.sh nav
```

检查有效 tuning（仿真已运行）：

```bash
docker exec smartcar-layered bash -c 'source /opt/ros/kinetic/setup.bash; rosparam get /single_goal_nav/effective_tuning'
```

## 当前选路与超时规则

1. 普通目标先求一条完整可行路线并保存，再用剩余时间尝试更少换向的路线。
2. 优先比较整条路线实际换向次数，次数相同时比较预计行驶时间。允许为了少换向绕远；这不是全局最少换向的数学保证。
3. 后续优化失败、超时不会丢弃已经成功的路线。只有没有完整可行路线时才失败；任务取消会使已有路线失效。
4. 两种特殊横移有线时只选距**最终目标**最近的线，不因失败切换到另一条线。没有线则进入普通模式。
5. 到①入口单独规划，确认停稳后，整体规划①→②退让点→最终目标；必须后两段都成功才行驶。②处不丢弃原末段重新规划。
6. ①、②均从原标称点沿车辆到达后的车尾方向，依次额外偏移0.3、0.6、0.9米。不是正负双向，也不额外检查零偏移。画线端点顺序不改变车尾方向。
7. ①发现无换向路线即采用；后两段各自无需内部换向的完整方案可提前采用。其余方案按整条路线实际换向排序，包括②处的换向。剩余预算还可优化已选入口路线或末段路线。目前未联合优化第二段和第三段的所有换向分配。

地图、车身碰撞和巡检深度约束始终有效。运行中发生意外恢复会按当前位置重新规划，并非无条件继续使用过期路径。

## 搜索时间与少换向参数

下表 `tuning.` 前缀表示位于 YAML 的 tuning 块，其他为顶层参数。单位秒、米，角度只有 `_deg` 后缀使用度。

| 参数 | 当前配置 | 作用与建议 |
|---|---:|---|
| `planning_timeout` | 5.0 | 基础规划预算，不代表所有模式只搜5秒 |
| `tuning.normal_planning_timeout` | 20.0 | NORMAL 模式预算至少20秒；STRAIGHT、TURN_90等独立模式不自动套用该值 |
| `tuning.line_entry_timeout` | 20.0 | 入口搜索总预算，多个候选和优化共享 |
| `tuning.line_suffix_timeout` | 15.0 | 后两段整体搜索共享预算 |
| `tuning.planner_switch_cost` | 0.45 | 搜索中一次前进/倒车换向的软代价。盲目增大会使有限时间内更难找到路 |
| `tuning.planner_short_segment_cost` | 0.15 | 对很短方向段之后的换向额外惩罚 |
| `tuning.planner_steer_cost` | 0.02 | 方向盘变化的软代价，不等同换向惩罚 |
| `tuning.planner_max_records` | 60000 | 搜索记录上限，也可能在时间耗尽前终止搜索 |
| `tuning.line_entry_steer_seconds` | 30.0 | 保留的转向评估辅助参数；当前入口/整段首要排序不再使用它，不应靠它减少换向 |

有效预算取基础预算与该阶段预算的较大值；恢复搜索耗尽时，从有效阶段预算倍增到最多20秒，原配置大于20秒时不降低。不是每次都等满预算，也不是硬实时保证，协作式检查可能略超时。

建议先保持代价默认值，比较入口位置和完整路径。小样本中提高换向软代价曾降低规划成功率，因此默认没有采用那些较强代价。不要同时改变预算、速度和几何后，把结果归因于某一个参数。

## 低代价线位置

| tuning参数 | 当前配置 | 含义 |
|---|---:|---|
| `line_entry_distance` | 1.5 | 从当前后轴中心在线上的投影，按横移规则生成①标称点的偏移距离 |
| `line_entry_search_span` | 0.9 | ①额外向车尾偏移为该值的1/3、2/3、1倍 |
| `line_retreat_distance` | 1.5 | 从最终前轴目标在线上的投影，向到达朝向车尾退让，生成②标称点 |
| `line_search_span` | 0.9 | ②额外向车尾偏移为该值的1/3、2/3、1倍 |

例如②实际尝试的退让距离是1.8、2.1、2.4米，而不是0.3、0.6、0.9米。阶段点内部使用后轴坐标；巡检目标使用前轴中点。候选有可能落在线段延长线上，最终受地图和车身碰撞检查限制。

启动默认 `load_saved_low_cost_lines: false`，本次自己画线；同一个 plan-test 会话可以复用已画线。更换地图应重新标定点位和线，旧文件的地图绑定不能当作已经适配新场地。

## 停车、恢复与容差

| 参数/规则 | 当前配置 | 说明 |
|---|---:|---|
| `goal_position_tolerance` | 0.04m | 普通目标基础规划容差；FINAL/后两段精确搜索使用不超过0.025m |
| `goal_heading_tolerance_deg` | 10° | 目标规划角度窗口；部分特殊模式另有阶段角度限制 |
| `tuning.final_finish_position` | 0.075m | 未放宽恢复模式时最终停车位置验收 |
| `tuning.final_finish_heading_deg` / `normal_finish_heading_deg` | 10° / 10° | 当前YAML最终停车朝向验收 |
| `tuning.stage_plan_position` / `stage_plan_heading_deg` | 0.06m / 5° | ①/②阶段搜索容差 |
| `tuning.stage_finish_position` / `stage_finish_heading_deg` | 0.12m / 10° | 阶段停稳后验收 |
| 恢复目标搜索 / 停车验收 | 0.08m / 0.10m | 以原前轴目标为圆心；目前是源码固定值，不是YAML参数 |

恢复候选留2厘米验收余量，不能保证真实位置误差一定小于10厘米：定位误差、控制误差仍存在。意外停止保留目标并检查新鲜传感器、TF和停稳状态，必要时返回上次确认停车点再重试；不是倒放历史控制指令。

正常导航持续恢复不设次数上限，`tuning.max_replans` 不能用来限制该恢复流程。plan-test不执行无限恢复。后台实验的“单目标12次恢复停止测试/900秒测试时限”属于测试器，不是正式导航的限制。

## 行驶速度、跟踪与碰撞

| 参数 | 当前配置 | 作用 |
|---|---:|---|
| `tuning.forward_speed` / `reverse_speed` | 0.07 / 0.045m/s | 路径速度上限，弯道、终点和反馈滞后会进一步减速 |
| `tuning.tracking_turn_allowance` | 0.015m | 转向未跟上时允许行进的距离量级；不是转向角容差 |
| `tracking_preview_distance` | 0.35m | 跟踪预瞄距离 |
| `tracking_lateral_gain` / `tracking_heading_gain` | 6.0 / 4.0 | 横向与朝向跟踪增益 |
| `planning_collision_margin` / `collision_margin` | 0.04 / 0.02m | 全局额外余量/执行安全余量 |
| `local_guard_distance` | 0.10m | 近程局部安全监督相关距离；不能据此把制动/响应碰撞检查关掉 |
| `tuning.tracking_max_error` | 0.20m | 偏离限制 |
| `scan_timeout` | 1.0s | 雷达有效期 |
| `tuning.feedback_timeout` / `pose_timeout` | 0.30 / 0.40s | 反馈和定位TF有效期 |

减少換向后再尝试提速；一次只小幅调整一项，并检查最大跟踪误差和恢复次数。扫描超时优先排查传感器/CPU/时间戳，不能仅放宽有效期掩盖问题。更换车辆必须重新量车身、轴距和最小转弯半径，不要缩小碰撞包络来换取通过率。

## 建议验证顺序

1. 校验YAML，重新启动导航。确认目标参考点仍为前轴中点。
2. 停止普通nav，再运行下面的plan-test，输入A/B组合、画线、确认开始：

```bash
bash ~/smartcar1/scripts/sim.sh plan-test
```

3. 固定地图、起点、A/B组合、线条、车速、噪声、资源和预算，对比换向次数、规划成功率及路径长度。plan-test虚拟到达不驱动车辆，不能证明跟踪精度。
4. 用真实Gazebo运动复测相同组合，记录完整任务成功率、真实耗时、恢复次数、最大跟踪误差。失败和恢复次数过多都需要处理，不能仅统计完成目标数。

离线筛查曾在6个记录起点中全部找到完整后两段路径，换向合计14→12、路径29.31→30.14米；这是保存可行解前后对比，不是全任务动态通过率。其余案例并未减少换向，实际速度和误差尚不能据此宣布改善。

实验脚本 `scripts/screen-anytime-planner.py`、`compare-steering-search.py`、`start-recovery-batch.py` 等依赖本机归档于 `data/logs/recovery_validation` 的地图绑定线条、任务和轨迹副本；这些运行数据不随源码提交。新机器应先生成自己的日志/点位/线条，或直接使用交互式plan-test；不要把缺失实验副本误认为导航启动失败。
