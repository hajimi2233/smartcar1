# 16 点巡检任务计划

当前实现由 `src/smartcar_mission/core/planner.c` 生成 A/B 任务访问顺序，`plan14.c` 将事件转换为坐标和朝向。原始算法来自归档版本，现在作为 `smartcar_mission` 的 C 可执行程序参与构建。归档的 26 点实现仅保留作参考。

## 标定

全部点位使用定位的 `map` 坐标系，目标参考点为前轴中点。依次标定：

| 次序 | 名称 | 含义 |
| --- | --- | --- |
| 1～10 | `inspect_1`～`inspect_10` | 五列通道各两个巡检点；每列奇数点在上侧，偶数点在下侧，列固定从右到左编号：最右列 1/2，最左列 9/10 |
| 11～14 | `outer_left_top`、`outer_left_bottom`、`outer_right_top`、`outer_right_bottom` | 左上、左下、右上、右下绕行点 |
| 15 | `start` | 任务开始位置参考，不下发为导航目标 |
| 16 | `end` | 任务终点 |

上、下、左、右是场地拓扑名称，不要求与地图坐标轴对齐。低编号端固定为右绕行点，高编号端固定为左绕行点，不做自动识别。保留原任务算法的起始约定：从上侧公共区域开始，按低编号侧的任务拓扑安排路线。起点坐标不会让算法重新优化起始侧；应把车放在这一约定的可达起始区域，再执行任务。

先启动定位并设置初始位姿，然后在独立终端运行：

```bash
bash ~/smartcar1/scripts/sim.sh points
```

在 RViz 选择 **Publish Point**（若旧窗口没有，在 Tools 中 Add → PublishPoint，话题设为 `/clicked_point`），按终端提示依次点击 16 个位置，不拖动方向。点位在地图上标定，标定不会发送导航目标。不应使用 2D Nav Goal 进行标定。

在另一个终端撤销最后一点或保存全部 16 点：

```bash
bash ~/smartcar1/scripts/sim.sh points-undo
bash ~/smartcar1/scripts/sim.sh points-save
```

默认写入宿主机 `data/tasks/points.csv`，表头为 `point_id,x,y`。可以直接编辑 CSV。保存会覆盖此文件；标定进程内的撤销不自动修改已保存文件。不同地图请使用不同文件，例如：

```bash
bash ~/smartcar1/scripts/sim.sh points /home/hajimi/smartcar/data/tasks/narrow_points.csv
```

## 方向生成规则

每列两个巡检点确定轴线。奇数点→偶数点定义向下方向，反向为向上方向；C 的进入侧决定每个巡检目标的车头朝向。通道内目标不依据“上一点→下一点”重新转向，从而保留倒车返回时应有的朝向。

绕行朝向由同侧上、下绕行点连线生成。从上到下依次发送上点、下点，两个目标都朝下；反向依次发送下点、上点，两个目标都朝上。终点朝向取上一个下发目标→终点；两点重合则沿用上一个朝向。这是明确的目标朝向规则，不是对任意地图最优航向的搜索。

每次任务下发十个巡检目标、必要的绕行目标和终点；16 是标定名称数，不是固定发送数量。绕行点可能重复使用。入口、退出和已巡检 A 点的静默返回事件不再额外下发，交由现有导航处理。

## 生成、预览和执行

首次加入 C 模块后需要 build，并在下一次 `start` 时使用新镜像。`start` 的场地切换或容器重建会重置定位，需要重新设置初始位姿。

```bash
bash ~/smartcar1/scripts/sim.sh build
bash ~/smartcar1/scripts/sim.sh start
```

只预览坐标和方向，不加载或执行导航：

```bash
bash ~/smartcar1/scripts/sim.sh plan A1A2B3B4A5B6A7A8B9B10 --preview
```

正式使用时保持 `start` 和 RViz 运行，另开终端启动现有导航并绘制本次需要的低代价线：

```bash
bash ~/smartcar1/scripts/sim.sh nav
```

再另开终端生成并加载多点任务，保持该终端运行（不要同时启动另一份 `multi`）：

```bash
bash ~/smartcar1/scripts/sim.sh plan A1A2B3B4A5B6A7A8B9B10
```

这一步只加载坐标队列。确认显示的坐标、朝向和任务顺序后，在另一终端开始或取消：

```bash
bash ~/smartcar1/scripts/sim.sh multi-execute
bash ~/smartcar1/scripts/sim.sh multi-cancel
```

指定单独保存的地图点位：

```bash
bash ~/smartcar1/scripts/sim.sh plan A1A2B3B4A5B6A7A8B9B10 /home/hajimi/smartcar/data/tasks/narrow_points.csv --preview
```

多点节点沿用已有的成功反馈推进逻辑。C 任务通过 `/single_nav/inspection_goal` 一条消息一起传递位置、朝向、目标编号和十个巡检区域的 A/B 类型及中心，避免坐标与区域约束错配；普通手动多点目标仍使用 `/move_base_simple/goal`。不增加退出目标或强制倒车阶段。到达巡检点即按现有导航成功反馈继续，没有新增拍照、播报或停留流程。坐标队列测试通过不表示实际场地的每一段都已验证可行。

## 交互式 plan + nav-test

保持 `start`、定位、RViz 运行，先关闭普通 `nav`、旧多点任务或键盘控制终端，再执行：

```bash
bash ~/smartcar1/scripts/sim.sh plan-test
```

程序会提示输入完整 A/B 配置，例如 `A1A2B3B4A5B6A7A8B9B10`，不自动采用示例。输入错误可重新输入；输入 `q` 退出。

输入正确后，会显示任务顺序及各点坐标、朝向，并启动原 `nav-test`（`planning-test`）和多点队列。**可在 RViz 画好本次低代价线，再回终端按回车开始测试；也可不画线，直接回车。** 没有低代价线时，两种横移均使用普通导航，不执行三段横移；有线时仍按最近线规划，失败不换线。 同一次 `plan-test` 程序内，下一组配置会自动复用本次画好的线；退出整个程序后，新启动不会恢复旧线。每次程序使用独立临时文件，不会覆盖平时保存的低代价线。需要修改时，在测试节点仍运行时执行 `bash ~/smartcar1/scripts/sim.sh lines-clear` 后重画，新线供后续各组复用。使用单点 RViz 配置即可看全局路径和 Virtual test pose；多点 RViz 配置还可以显示编号目标。

这是逐目标的全局规划与虚拟到达测试，实际车保持停止。第一段从当前定位对齐的仿真位姿开始，之后接着上一段的虚拟终点规划；CSV 的 `start` 不会传送车辆或替代当前起始位姿。测试沿用导航的角度容差及特殊横移流程。

终端显示 `SUCCEEDED: all ... points` 表示整组全局规划及流程通过；`FAILED at goal N/...` 显示失败目标编号与原因。结果写入同目录的 `last_plan_test.json`，默认宿主机路径为 `data/tasks/last_plan_test.json`。每轮结束后释放测试节点，可继续输入下一组配置；Ctrl+C 退出不会关闭 `start` 的仿真和定位。

指定其他点位文件：

```bash
bash ~/smartcar1/scripts/sim.sh plan-test /home/hajimi/smartcar/data/tasks/narrow_points.csv
```

单独启动原规划测试可用 `bash ~/smartcar1/scripts/sim.sh nav-test`，它是 `planning-test` 的别名。全局规划测试成功不代表实际运动误差和局部控制已验证。

## 目标预览

RViz 的 Task goals and headings 显示 `/multi_nav/points`（MarkerArray）：黄色为当前目标，青色为下一个目标，其他目标为灰色，失败目标为红色。箭头从前轴中点出发表示目标车头朝向；文字包含任务序号、名称、坐标和角度。加载队列后即显示，发送目标前更新。失败后保留队列标记；plan-test 等待回车后才关闭本轮，便于检查。旧 RViz 窗口可手动 Add → MarkerArray → `/multi_nav/points`，或重新打开 RViz。

## 从原 14 点文件升级

先 Ctrl+C 关闭原 `points` 标定终端，执行：

```bash
bash ~/smartcar1/scripts/sim.sh points-outer
```

按提示用 Publish Point 点击左上、左下、右上、右下四个外围点，再在另一终端执行 `points-save`。巡检点、起点、终点原坐标保留；保存前自动备份原 CSV。旧左右中间点不能直接作为上/下端点，不会自动推算新坐标。完整文件现在必须有 16 个命名点。`plan-test` 用法不变。

低代价线仍统一按最终目标的前轴中点选择最近线，不为绕行目标增加特殊选线规则。

## 巡检深度边界

不需要画框，十个巡检点分别生成地图坐标轴对齐的 0.8 m × 0.8 m 区域（x、y 各 ±0.4 m）。参考点为前轴中点，车身碰撞检查仍单独生效。

只应用两类限制：

1. 前往任意 A/B 巡检目标时，向下进入限制前轴 y 不得到达/越过中心 y−0.4 m；向上进入限制 y 不得到达/越过中心 y+0.4 m。边界本身按阻挡处理，检查覆盖整条路径，不能先越界再返回。
2. 发起下一个目标时，如果当前前轴中点仍在 B 区域内，保留进入该 B 区域时的深度方向。即使下一目标是终点、绕行点或另一个巡检点也生效；如果下一目标也是巡检点，同时应用它的目标深度限制。原 B 限制在本次目标的全部规划阶段和重规划中保持，下一目标开始时再根据所在区域更新。

限制的横向范围是中心 x±0.4 m，在这一宽度内，深度边界后方均不可进入；不把边界延长到整张地图。进入侧可接近、可退回，横向范围以外仍由地图墙体和车身碰撞检查约束。没有额外禁止所有未巡检区域，也不会单独生成退出点。无合规路径则失败，不放松深度边界。

全局规划的路径采样与相邻采样之间的前轴扫掠都检查边界；局部运动响应和制动轨迹也使用同一约束。`nav-test` 共用全局约束，但虚拟到达不验证真实执行误差。日志 `DEPTH_LIMIT` 显示当前区域、上限/下限和边界坐标；碰撞诊断会标记 `INSPECTION_DEPTH_inspect_N`。

C 生成结果新增 `inspection_type`。旧的 JSONL 任务文件需要重新生成；16 点 CSV 不变，不需重新标定。首次从一个已处于 B 区域的位置开始、且没有先前进入记录时，以当前车头上下朝向确定进入侧；车头接近水平、无法可靠判定时拒绝该任务。
