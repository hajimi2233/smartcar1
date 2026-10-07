# 实车调参入口

本说明覆盖车辆参考点、几何和基础参数。当前少换向搜索、恢复策略、两阶段低代价线规划的默认值与调试步骤，见 [导航调参说明书](NAVIGATION_TUNING.md)。默认值以 config/navigation.yaml 和 config/navigation-real.yaml 为准。

## 修改和生效

本结构版的仿真参数为 `config/navigation.yaml`，实车模板为 `config/navigation-real.yaml`。启动入口和首次编译见 [逐步复现](REPRODUCE.md)。修改配置后停止当前导航，再重新运行 single/multi/planning-test，不需要重建镜像。行驶中不热更新参数。

仿真 `scripts/sim.sh` 自动选择仿真配置；本机 `scripts/robot.sh` 默认选择实车模板。可用环境变量 SMARTCAR_NAV_CONFIG 指定其他绝对配置路径。两套文件初始几何值相同，实车模板的尺寸仍需测量。

配置校验：`python scripts/check-navigation-config.py config/navigation-real.yaml`（需要 PyYAML）。ROS 中用 `rosparam get /single_goal_nav/effective_tuning` 查看生效值。

## 三种参考点分别设置

所有偏移均相对于后轴中心，车辆自身坐标 **x 向前、y 向左，单位米**。朝向始终表示车身朝向，不表示路径切线（倒车也不加 180°）。定位坐标系的轴方向必须与车身一致，传感器旋转外参应在 TF 中处理。

| 参数 | 控制什么 | 默认 |
|---|---|---|
| `tuning.base_reference` | 定位 TF 的 base 原点位于车上哪里 | custom，x=0.31，y=0 |
| `tuning.goal_reference` | RViz / 单点 / 多点目标位置代表车上哪里 | front_axle |
| `tuning.path_reference` | 发布的全局、局部 Path 和虚拟位置代表车上哪里 | rear_axle |

每项可选 `rear_axle`、`front_axle`、`custom`。前轴位置自动使用 `wheelbase`；custom 使用对应 `*_offset_x`、`*_offset_y`，其他模式忽略这两个偏移。

例如以后目标和输出路径都要表示车体中心，中心在后轴前方 0.30 m：修改现有 `tuning` 中的字段，不要添加第二个 tuning 块：

```yaml
  goal_reference: custom
  goal_offset_x: 0.30
  goal_offset_y: 0.0
  path_reference: custom
  path_offset_x: 0.30
  path_offset_y: 0.0
```

若实车定位直接提供后轴中心，只改 `base_reference: rear_axle`。改目标参考点后，已有目标坐标的物理含义会改变，需要重新标点或换算目标。

内部阿克曼运动学、碰撞检测、低代价线阶段点和跟踪仍统一在后轴计算，然后转换输入/输出参考点。这保证改变显示点或定位原点不会错误改变车辆运动模型。`path_reference` 改变发布的路径位置，不改变控制器参考线；外部消费者若接收这些 Path，应按同一偏移换回后轴。当前内部 curvature 和速度均定义于后轴，不是任意参考点的轨迹曲率/点速度。

## 车辆几何与底盘对接

| 字段（除注明外均在 tuning 内） | 含义 |
|---|---|
| wheelbase / front_track / wheel_radius | 轴距、前轮轮距、轮半径；同时影响关节反馈解码、转角换算与局部预测 |
| physical_min_radius / global_min_radius | 实车最小可用半径、全局规划允许的半径下限 |
| 顶层 turn_radius / local_turn_radius | 实际全局/局部规划半径，默认 1.3 / 1.2 m |
| turn90_primitive_file | 可选的右转 90°标定 JSON；导航会自动镜像成左转，并在实测轨迹末端继续普通规划。留空则使用原有 Hybrid A* 90°规划 |
| body_front / body_rear / body_half_width | 区域碰撞检查的车身前伸、后伸、半宽，长度都取正值 |
| collision_center_x / collision_half_length / collision_half_width | 栅格地图和雷达碰撞矩形中心及半尺寸 |
| boundary_half_length | 原版地图边缘代价使用的半长度 |
| tire_half_width / region_margin | 前轮包络半宽、区域检查余量 |
| 顶层 collision_margin | 栅格碰撞余量，原有最低 0.02 m 校验保留 |
| map_frame / odom_frame / base_frame / truth_frame | 导航使用的 TF 坐标系名称 |
| 顶层 cmd_topic / joint_topic / actuator_model_param | 指令话题、关节反馈话题、执行器模型参数路径 |

原版区域轮廓与栅格碰撞矩形略有差异，因此这里分别保留默认值，未自动缩小或统一。换车时应一起核对这两组尺寸。这里只提取碰撞相关参数，不调整碰撞策略。

执行器模型仍要求 version=1，包含 wheelbase、front_track、wheel_radius、acceleration、braking、steer_rate；前三项必须与本文件一致，否则启动报错，防止一边用仿真尺寸、一边用新尺寸。实车可以把模型放到 `/vehicle/actuator_model` 并修改参数路径。真实加减速、转向速率仍由执行器模型提供。

当前 `cmd_topic` 使用 Twist：linear.x 是后轴纵向速度（m/s），angular.z 是车体角速度（rad/s），不是转角。`joint_topic` 仍按现有四个关节名称解码。若实车使用串口转角、编码器等不同协议，仍需底盘桥接，不能仅修改话题名就当作已完成硬件适配。修改导航轴距不会修改 Gazebo URDF 或物理模型。

## 经常调整的参数

| 类别 | 字段 | 说明 |
|---|---|---|
| 阶段距离 | line_entry_distance / line_retreat_distance | 默认各 1.5 m，分别影响线上第一阶段位置和第二阶段退让距离 |
| 规划到达 | 顶层 goal_position_tolerance / goal_heading_tolerance_deg | 规划终点允许误差；与停车确认容差不同 |
| 阶段规划 | stage_plan_position / stage_plan_heading_deg / maneuver_plan_heading_deg | 阶段规划位置、角度和特殊动作角度上限 |
| 停车确认 | stage_finish_* / final_finish_* / normal_finish_heading_deg | 阶段或最终停止后的确认误差 |
| 提前停车 | stage_early_* / final_early_* | 开始请求停车的阈值，应不大于停车确认容差 |
| 速度 | forward_speed / reverse_speed | 全局路径的正常前后进速度上限 |
| 跟踪速度 | tracking_forward_speed / tracking_reverse_speed / tracking_min_speed / approach_speed_gain | 控制器候选速度、接近终点减速 |
| 跟踪增益 | 顶层 tracking_lateral_gain / tracking_heading_gain / tracking_preview_distance / steering_rate | 横向、航向反馈、曲率变化预瞄和指令转角变化限制 |
| 时序 | control_period / max_control_dt / feedback_timeout / pose_timeout / truth_timeout | 控制循环等待时间、最大 ROS 时间间隔、反馈有效期 |
| 停车/进度 | stop_window / stop_position_span / stop_heading_span / stop_timeout / progress_timeout / max_replans | 停稳确认、无进展等待和重规划次数 |
| 局部预测 | prediction_step / rollout_steps / rollout_dt / steering_sample_offset | 积分步长、预测次数、后续预测间隔、转角候选偏移 |
| 偏离限制 | tracking_max_error / tracking_recover_error | 同时用于局部候选检查和导航偏离检查 |
| 雷达 | scan_obstacle_range / scan_stride / 顶层 scan_timeout | 障碍点取用距离、抽样间隔与数据有效期 |
| 靠墙混合 | wall_straight_weight / wall_maneuver_weight / wall_normal_weight / wall_gain / wall_max_angle_deg | 各模式的墙体修正比例；默认特殊动作仍为 0 |
| 测试回放 | replay_forward_speed / replay_reverse_speed / replay_min_speed / 顶层 test_replay_speed_scale | 关闭虚拟到达时的真值回放速度；虚拟到达模式不驱动车辆 |

带 `_deg` 的字段用度，其他角度字段用弧度，距离用米、速度用米/秒、时间用秒。配置中保留了一些算法内部常数，未把所有数学常数和可视化尺寸变成调参项。当前搜索代价和低代价线规则见导航调参说明书。

## 历史参数提取验证记录（非当前版本验收）

新增测试覆盖非默认轴距的转向反馈/预测一致性、前后轴及自定义横向偏移的转换、目标规划、速度和阶段距离、碰撞矩形以及错误配置拒绝。未在本机运行 ROS/Gazebo，也未完成实车桥接验收。原版闭环终点测试的已知失败不属于本次参数提取的修复范围。

本次 Windows/Python 3 离线验证：78 项测试，77 项通过；唯一失败为原版已存在的 `test_planner_and_closed_loop_reach_front_axle_pose`。新增 9 项参数专项测试全部通过。配置默认值一致性、YAML/launch XML 解析及 Python 语法检查通过；ROS Kinetic/Python 2 与 Gazebo 运行仍待目标环境确认。
