# 摄像头画面与 patrol 任务联动：已部署的模拟模式

状态：**2026-10-17 已部署到车辆。车端 Python 2 的34项联动检查、94项导航回归通过；已验证真实摄像头取图和后台启动，以及独立 ROS 环境中的任务话题切换。未做真实巡检行驶或图形窗口现场验收。**

## 快捷打开

在车辆图形桌面终端输入：

```bash
~/camera
```

默认打开巡检联动模拟模式：真实框取、按 patrol 内容显示类别、每秒约20%概率加入其他类别。原始真实视觉模式：

```bash
~/camera real
```

SSH 不弹窗口时用 `~/camera --headless` 或 `~/camera real --headless`。按 q / Esc / Ctrl+C 关闭图形窗口。快捷指令会阻止同时再开一个同类窗口占用相机；它不会自动启动导航或执行巡检。

这个模式使用真实摄像头画面和原有图片框取，但框内的类别文字来自 `patrol load` 的任务描述，并按配置加入随机错误。画面标题、框标签、底部以及专用结果话题均明确标注 `SIMULATED` / `SIM`。它用于联动演示，不能把它的输出当成真实视觉识别效果。

原 `~/vision_v5/run.sh`、识别算法文件、分类阈值、真实分类日志及巡检语音逻辑均保留。普通视觉命令不会自动启用模拟。任务的路线、目标坐标、全局规划、车辆控制和到点判定没有修改。

## 实际效果

假设任务从 1 号前往 3 号，且是需要先退出旧通道的不掉头横移：

| 阶段 | 画面所属点 |
|---|---|
| 前往 1 号点 | 1 号 |
| 1 号完成，导航收到 3 号目标 | 仍为 1 号，等待判定导航模式 |
| 横移第一段，倒车退出 1 号所在通道 | 仍为 1 号，显示 EXIT_WAIT |
| 第一个退出点已停稳，导航确认后准备沿公共区域横移 | 切为 3 号 |
| 横移并进入 3 号 | 3 号 |

切换使用已有导航在验收退出点后发出的 `LINE_CONFIRMED` 阶段事件，不根据时间、距离猜测，也不会在刚下发下一个目标时立即切换。每个事件携带本次运行和本个目标的唯一标识；上一目标延迟消息、旧任务残留消息不会解锁新目标。

普通直行和转弯目标，在导航确认其模式后切到新点。若已经判断为横移，后来规划退回普通模式，也不会借此提前切换；没有可靠的首退出点事件时，保守保持旧点，最迟在确认到达新巡检点时切换。`LATERAL_TURN_180` 也采用相同的首退出点门槛。

队列取消、失败、完成或程序退出会清除当前点。心跳丢失超过 3 秒，停止显示按任务注入的类别，并显示 `CONTEXT_STALE`，避免队列已经断开却一直挂着旧点。

## patrol load 输入怎样对应类别

输入中的十个位置依次对应 1～10 号巡检点，**数字是图片类别，不是点号**：

| 编号 | 类别 |
|---|---|
| 1 | 火源 FIRE |
| 2 | 水源 WATER |
| 3 | 行人 PEDESTRIAN |
| 4 | 道路 ROAD |

A 为正常 NORMAL，B 为异常 ABNORMAL。例如：

```bash
~/patrol load A1A2A3A4B1B2B3B4A1B4
```

其中 1 号点是正常火源，2 号点是正常水源，5 号点是异常火源。执行顺序仍由原程序规划；画面跟随实际当前目标的点号，不按数字从小到大自行轮换。

`patrol load` 仍只加载任务；按原流程 `~/patrol run` 才开始执行。不要为了测试文字而直接让车执行未知路线；本包的离线测试不发送任何车辆指令。

## 随机变化

默认每 **1 秒**抽样一次：80% 显示当前点预设类别与状态，20% 从其他三种类别中随机选一个，状态也随机选正常/异常。这是每次抽样的概率，不保证短时间内恰好五次错一次。抽样结果保持到下一个周期，不会在每帧之间高速闪烁；点号切换时立即重新取样，不必等旧周期结束。

概率可以调：

```bash
--demo-error-percent 20
--demo-update-seconds 1.0
```

百分比范围 0～100；更新周期 0.2～10 秒。`--demo-seed 123` 可固定随机序列以便复现；不提供 seed 时每次运行会变化。

图片框仍由真实检测器获得；没有检测到图片时不会凭空造一个框，也不会伪造相机已看到图片的记录。随机错误只属于模拟画面的类别，不修改原识别结果、不改变语音或实际任务数据。

## 安装记录与今后迁移

本车已经安装，**不需要再执行下面的安装命令**。工具及收据在 `~/patrol_vision_demo_20261017/`；这些步骤供以后核对或迁移使用。

把整个文件夹复制到：

`/home/smartcar/patrol_vision_demo_20261017/`

先停止当前 patrol、导航、车辆桥接和摄像头窗口。只停止这些应用，定位和雷达配置不需要修改。让车保持静止，车端执行：

```bash
source ~/smartcar_2026_ws/scripts/source_env.sh
cd ~/patrol_vision_demo_20261017
python install.py
```

先看到 `PACKAGE_HASHES_PASSED`、`PREFLIGHT_PASSED`，再安装：

```bash
python install.py --apply
```

安装器只替换两个节点文件、添加联动辅助模块和独立视觉启动器，并补充辅助模块的 CMake 复制规则；现有 devel 辅助模块已一并处理，无需为本次 Python 修改重新编译。它不会启动 ROS、摄像头、导航或底盘，也不会下发目标。

导航基线是 2026-10-17 刚部署并验证过的版本；任务节点基线来自本地 2026-10-16 交付快照。两者已在本次安装前重连车端核对，文件哈希一致。如果今后提示 `Vehicle source changed`，应取回新文件合并，不要绕过检查强装。

## 安装后使用

重新启动原定位、导航和任务，正常使用 `~/patrol load ...`。另开车辆图形桌面终端打开模拟摄像头窗口：

```bash
bash ~/vision_v5/run_patrol_demo.sh --source 0 --fps 5
```

需要调整随机行为：

```bash
bash ~/vision_v5/run_patrol_demo.sh --source 0 --fps 5 \
  --demo-error-percent 20 --demo-update-seconds 1.0
```

摄像头方向和原命令一样默认转 180°，仍可用 `--rotate 0`。按 q / Esc 或 Ctrl+C 关闭。不要同时运行原 `run.sh` 和新窗口占用同一摄像头。`--headless` 可以沿用，此时专用模拟结果话题仍会更新，但没有窗口。

仅加载还没有执行时，显示 `PATROL_INACTIVE`；执行后显示 `SIM P1 NORMAL_FIRE` 等文字。需要恢复真实视觉窗口时，关闭模拟窗口，再运行原命令：

```bash
bash ~/vision_v5/run.sh --source 0 --fps 5
```

## 检查联动

```bash
rostopic echo /inspection/patrol_context
rostopic echo /single_nav/patrol_stage
rostopic echo /inspection/demo_preview
```

- `patrol_context` 是任务元数据：当前显示点、等待切换点、阶段、运行/目标标识，每 0.5 秒更新。
- `patrol_stage` 是导航原有状态的带目标标识副本；不改变导航状态机。
- `demo_preview` 是明确带 `simulated:true`、`source:SIMULATED_PATROL_LABEL` 的模拟类别。它只在真实图片框锁定时发布，不能接到正式识别或巡检结果话题。

安装后任务节点和导航节点都必须重启，旧进程不会自动获得新接口。若一直 `EXIT_WAIT`，先核对 `patrol_stage` 是否有对应目标标识的第一段退出确认；不要用延时强行跳过这个条件。

## 验证范围与限制

新增检查包含：首点绑定、横移退出门槛、旧消息隔离、取消/失败/心跳超时、每秒保持结果、随机频率、框显示不修改真实分类、任务坐标和约束保持一致、安装失败恢复、拒绝覆盖后续修改。

另附原导航 94 项回归，已将新的导航节点候选放进回归目录。其全局规划、跟踪、换向及退出逻辑保持原值；新增导航修改仅是可选元数据。

历史离线结果见 `results/local_validation.json`、`results/checks.log` 和 `results/navigation_regression.log`。本次车端结果以 `results/deployment_result.json`、`vehicle_validation.json`、`camera_verification.json`、`ros_linkage_verification.json` 为准。

车端为 Python 2.7.12 / OpenCV 3.3.1-dev。真实相机连续读取8帧、640×480，通过原检测器及模拟叠字绘制；两种快捷指令的后台启动均通过。当前现场画面没有稳定锁定巡检贴图，因此没有验证真实贴图的分类准确率。

ROS 话题联动用独立11329 master和合成的到点/退出状态验证，未改变正在工作的11311 master，不是实车路线验收。新导航和定位节点也在禁用驱动的独立环境成功启动，原低代价线正常载入。所有检查进程和摄像头测试均已退出；原雷达、定位和底盘会话未被终止。

发布包已加入 Linux 可执行文件 LF 换行校验，避免 Windows 文本换行使 rosrun 报 `python\r`。

## 回退

先停止 patrol、导航、桥接和摄像头窗口，再执行：

```bash
source ~/smartcar_2026_ws/scripts/source_env.sh
cd ~/patrol_vision_demo_20261017
python install.py --rollback
```

备份位于工作区 `data/backups/patrol_preview_时间_进程号/`，收据为本目录 `results/installation.json`。如果安装后手工改过相关文件，回退器会拒绝覆盖，需要对照备份合并。

本包属于 10月17日导航部署之后的追加修改。以后若回退整套导航，应先回退本包，再回退之前的部署；原部署的逐文件哈希检查会把本包视为后续修改，这是预期保护。

`build_candidates.py`、`build_regression.py` 用于本机重建，依赖原工作区，车端安装不需要运行它们。原来的地图、点位和低代价线没有改动。
