# 仿真轮距与雷达融合验证

## 已实现的内容

新增可选 `wheel_fusion:=true`，在现有墙体定位中使用带时间戳的有符号累计脉冲：

- 脉冲约束后轴中心的短时纵向距离，雷达里程计提供朝向变化；圆弧按中间朝向和弧长/弦长关系积分，转换时考虑后轴中心到 `base_footprint` 的 0.31 m 偏移。
- 墙面匹配同时优化地图表面残差与轮距增量残差。轮距不提供独立朝向、绝对位置或全局重定位信息。
- 轮距数据跨扫描时刻插值，禁止向数据范围外外推；检查缺帧、逆序、异常跳变和编码器重启。异常时显示 `FALLBACK`，这一帧使用雷达运动预测，不把错误计数当成距离。
- 同时启动独立纯雷达对照节点。两种算法使用相同地图、墙体选择、雷达扫描、初始位姿和匹配参数，只有融合节点发布 `map→odom`，对照节点仅发布估计位姿。
- 评估节点按相同扫描时间戳比较两种结果与仿真真值，并记录 JSONL。它只读数据，不能发布定位、初始位姿、TF 或速度。

这是墙体匹配器的轮距增量融合实验版，**不是已经完成的 AMCL 粒子滤波融合**。输出仍按雷达扫描频率更新，当前约 10 Hz；高频脉冲用于提高每次距离积分的分辨率，不意味着已经实现高频地图位姿输出。

## 仿真传感器模型

`sim_wheel_pulses.py` 将 Gazebo 真值中的后轴中心位移积分为脉冲，仅发布一个标量计数和时间戳；不把真值姿态或坐标传给定位器。默认 10000 脉冲/米是测试值，不是实车已标定数值。

支持比例误差 `scale_error:=0.0`、`0.05`、`-0.05`，以及计数量化。未模拟真实车轮空转、瞬时打滑、硬件传输协议、机械回差和轮径变化。它是基于实际运动的合成轮距传感器，用来检验融合方法，不等价于真实黑箱编码器模型。

接线边界：

- 真值 → 模拟脉冲传感器 → `/wheel/pulses` → 融合定位。
- 真值 → 只读评估节点。
- 雷达 → 激光里程计/墙体匹配 → 融合定位与纯雷达对照。

## 手动运行

首次使用新代码需要构建镜像。若已由开发任务完成镜像构建，不必重复。先停车并关闭旧定位、键盘控制：

```bash
bash smartcar1/scripts/sim.sh build
bash smartcar1/scripts/sim.sh start
```

开一个终端启动模拟脉冲和评估记录，先测试理想标定：

```bash
bash smartcar1/scripts/sim.sh wheel-test scale_error:=0.0
```

另开终端启动轮距融合和纯雷达对照：

```bash
bash smartcar1/scripts/sim.sh localization /home/hajimi/smartcar/data/maps/slam_current/map.yaml wall_features:=true wheel_fusion:=true
```

打开 RViz，确认墙体已加载/保存，并用 2D Pose Estimate 设置初始位姿。未设置通道框或墙体时请先按 WALL_FEATURES.md 操作。

```bash
bash smartcar1/scripts/sim.sh rviz
bash smartcar1/scripts/sim.sh wall-status
bash smartcar1/scripts/sim.sh wheel-results
```

这些持续运行命令分别使用独立终端。键盘控制：

```bash
bash smartcar1/scripts/sim.sh keyboard-sim
```

完成一组测试后停车，重启 `wheel-test` 和定位进行下一组，将 `scale_error` 改成 `0.05` 或 `-0.05`；确保可比较的初始位置、墙体选择、路线和速度。传感器重启具有新的 epoch，不会把累计计数清零当成倒车。

默认评估假设 `map` 与 `sim_world` 的平面坐标一致。**自己建的地图未必满足这个条件**，必须提供固定且独立已知的 `truth_to_map:='[x,y,yaw]'`，否则绝对误差无意义。脚本不会用每次定位结果对齐真值来掩盖误差。自动测试使用原生 `sim_field_v1` 地图，其坐标与世界一致；手动使用 `slam_current` 时需自行核对地图对齐。

结果保存在本机 `smartcar1/data/logs/wheel_eval_时间.jsonl`。`wheel-results` 显示位置 RMSE、相对初始位姿的位移误差 RMSE、朝向 RMSE、最大位置误差、输出时间戳年龄。时间戳年龄包含输出处理等待，不是通过轨迹拟合测出的完整动态延迟。统计只覆盖双方都有有效位姿的共同时间戳；应同时检查两个节点的状态，不能用少量成功样本掩盖定位掉线。

## 参数与接口

`config/localization.yaml` 的 `wall_localizer`：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `ticks_per_meter` | 10000 | 脉冲/米，必须与传感器标定一致 |
| `rear_offset` | 0.31 m | `base_footprint` 在后轴中心前方的距离 |
| `wheel_sigma_floor` | 0.005 m | 每段轮距约束的不确定性下限 |
| `wheel_relative_sigma` | 0.05 | 随距离增长的不确定性项 |

每段标准差为 `wheel_sigma_floor + wheel_relative_sigma * abs(distance)`。固定的比例误差会连续累积，并不会因为设置 5% 不确定性就自动校准。标定差时应增大轮距不确定性，不能总是给予极强约束。地图表面残差的权重按现有约 5 cm 尺度归一化；当前协方差是局部优化近似，不是完整累计误差滤波，不能当作经过标定的绝对位置置信度。

`/wheel/pulses` 使用现有 `sensor_msgs/JointState` 作为实验传输容器：

- `header.stamp`：采样时间，与 ROS 雷达时钟同域。
- `header.frame_id`：计数器 epoch 标识；不是 TF 坐标系。重启/清零必须改变标识。
- `name = ['rear_center_encoder_ticks']`。
- `position = [累计有符号整数脉冲数]`，正进负退；此字段在该专用话题上不是弧度。
- `velocity`、`effort` 留空。整数计数绝对值限制小于 2^53。

实车适配时可换成专用消息类型；目前保持这个接口即可复用融合部分。仅有高速脉冲而没有同步采样时钟，不能直接照搬。

`/wall_localization/status` 中 `wheel_state` 为 `DISABLED`、`WARMUP`、`FUSED` 或 `FALLBACK: 原因`。`map_rank` 指地图墙面独立提供的约束秩；单面墙依然报告 `PARTIAL`，即使轮距补足了短时增量约束，也不冒充完整绝对定位。

纯雷达对照发布 `/lidar_baseline/pose` 与 `/lidar_baseline/status`。可在 RViz 增加 PoseWithCovariance 显示 `/lidar_baseline/pose`，与 `/wall_localization/pose` 对照。

## 自动验证

- `tests/test_wheel_distance.py`：正反转、插值、断流、epoch、异常跳变、圆弧积分、后轴参考点、比例误差、匹配约束和校正上限。
- `tests/ros_wheel_fusion.py`：隔离 ROS 中的模拟脉冲、双定位、单一 TF 发布、前进倒车、断流恢复和同步评估。
- `tests/gazebo_wheel_experiment.py --isolated-sim`：**只在独立容器和独立 ROS master 中运行**，会自动发送仿真速度。三组 0%、+5%、−5% 比例误差，覆盖静止、直线前进/倒车、圆弧前进/倒车、停止。`WHEEL_PARALLEL_ONLY=1` 改为只选一面平行墙并加长直线行程。

同一组内两种算法共享相同扫描，跨组仍包含随机噪声及物理仿真差异。实验中没有读取 Gazebo 真值来给估计器实时设置位置；初始位姿来自固定已知的场景重置位置，并有相同的小偏差。真实位姿只用于脉冲生成和评估。
