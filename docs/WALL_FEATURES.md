# 画线膨胀选墙：通道内外优先定位

## 选择规则

一条手画线段向四周膨胀成区域，**区域覆盖的地图墙体都被选中**。默认膨胀半径 0.30 m（总宽 0.60 m），两端为半圆。按占用栅格中心是否落在区域内选择；不扩展到范围外整堵连通墙。

厚墙的内部栅格与两侧表面、多堵墙、断开的墙段都保留，不做“挑选唯一墙面”，不会因为两侧表面同时选中而报歧义。空闲和未知栅格不变成墙；缺口不填补；手画线本身不作为定位目标。

定位使用选中墙体中邻接已知空闲区域、局部表面方向可估计的边界栅格。按照预测位置将雷达点关联到这些地图表面，做点到局部表面的匹配。墙体内部用于表示选择范围，不作为雷达应打中的表面。所有目标来自地图，没有全图 AMCL 二次校正。边界以栅格中心表达，精度仍受地图分辨率、建图误差和初始化影响。

通道内/外分别使用 `inside`/`outside` 两组。依据已保存通道框和定位参考点切换；边界算内，切换确认时间默认 0.3 秒。一条选择线内的多个表面均可参与匹配，而非按点数只取一个表面。重复墙附近仍可能出现**定位关联错误**，与“选择区域允许包含多面墙”是不同问题。

## 绘制、预览与保存

RViz Fixed Frame 使用 `map`。工具 **Draw Localization Walls**（W）每两个点击点生成一条选择线，可以连续画多条。

```bash
bash smartcar1/scripts/sim.sh walls inside
# 用画墙工具画通道内优先区域的中心线
bash smartcar1/scripts/sim.sh walls outside
# 画通道外优先区域的中心线
bash smartcar1/scripts/sim.sh walls preview
```

RViz 中：

- 细橙色/蓝色线和编号：选择线。
- 线周围的圆头轮廓：膨胀区域。
- 绿色栅格：预览选中的全部地图墙体。
- 橙色/蓝色栅格：保存后通道内/外使用的墙体。

预览确认后：

```bash
bash smartcar1/scripts/sim.sh walls save
```

范围太小或太大，可以设置半径，单位米。此指令修改**两组全部草稿线**的半径，并作为接下来新画线的默认半径，保存前不影响定位：

```bash
bash smartcar1/scripts/sim.sh walls radius 0.30
bash smartcar1/scripts/sim.sh walls preview
bash smartcar1/scripts/sim.sh walls save
```

| 指令（接在 `bash smartcar1/scripts/sim.sh` 后） | 作用 |
| --- | --- |
| `walls inside` / `walls outside` | 选择绘制组 |
| `walls list` | 查看选择线、半径、选中栅格数量、表面采样数量和错误 |
| `walls undo` | 撤销待完成端点，或当前组最后一条线 |
| `walls delete outside 2` | 删除外侧组第 2 条草稿线 |
| `walls clear inside` / `walls clear outside` / `walls clear all` | 清空对应草稿 |
| `walls radius 0.30` | 设置全部草稿及新线的膨胀半径 |
| `walls preview` | 预览，不改变定位目标 |
| `walls save` | 重新选取墙体，全部成功后原子保存并激活 |
| `walls cancel` | 放弃未保存的线编辑，恢复生效选择线 |
| `walls load` | 加载文件并从当前地图重建选中墙体 |
| `wall-status` | 查看当前使用的组和定位匹配状态 |

添加、删除和半径变化均需保存才生效。仅在没有覆盖占用栅格、没有可用已知空闲侧表面、数据无效等情况下拒绝保存；失败不覆盖原文件和生效配置。默认半径也可在 `config/localization.yaml` 的 `wall_editor.selection_radius` 配置；已有线保存了各自半径，不会因默认值变化而改变。

## 旧数据与运行版本

旧版 schema 1 的墙线、schema 2 的选墙线会直接作为膨胀选择线重新处理，不再执行唯一墙面提取。成功保存为 schema 3，包含选择线、各自半径、选中的地图栅格和用于匹配的表面采样；每次加载均从当前地图重建，不信任陈旧目标。文件绑定地图指纹，默认保存在地图 YAML 同目录 `localization_walls.json`；`external_region.json` 是通道框。Docker 挂载的数据目录保留这些文件。

新代码必须进入运行容器才生效。停车并退出键盘控制，关闭旧定位/RViz 后：

```bash
bash smartcar1/scripts/sim.sh build
bash smartcar1/scripts/sim.sh start
```

另开终端启动定位（路径替换为实际地图）：

```bash
bash smartcar1/scripts/sim.sh localization /home/hajimi/smartcar/data/maps/slam_current/map.yaml wall_features:=true
```

再开终端启动 RViz，预览并保存：

```bash
bash smartcar1/scripts/sim.sh rviz
# 以下命令在另一个终端运行
bash smartcar1/scripts/sim.sh walls preview
bash smartcar1/scripts/sim.sh walls save
```

使用 RViz **2D Pose Estimate** 设置静止小车的大致位置与朝向。没有通道框时用 `region begin`、Draw Region 四角画框、`region save` 创建。墙模式会一起启动通道工具，无需重复启动 `corridor`。实车将 `sim.sh` 换成 `robot.sh`，先 source 环境并编译。

## 定位边界

`wall_features:=true` 模式不运行 AMCL；`map→odom` 仅由墙体定位发布。激光里程计负责连续运动估计，人工初始位姿和选中地图墙面负责地图定位。匹配与选墙是两个距离参数：`wall_editor.selection_radius` 决定选中哪些地图墙，`wall_localizer.association_distance` 决定扫描点在预测位置附近可以关联多远（默认 0.30 m）。不要把增大选墙范围当作扩大重定位能力。

一面或多面平行墙一般只能约束法向位移和朝向，沿墙方向依赖里程计，状态为 `PARTIAL`。足够的不平行表面可得到 `MATCHED`；这不等于已经验证了真实位置准确性。缺特征短暂进入 `ODOM_ONLY`，超过 `coast_timeout`（默认 2 秒）停止更新地图定位，通道判断随后为 `UNKNOWN`。其他状态包括 `WAIT_INITIAL_POSE`、`LOST`、`NO_FRESH_SCAN`。

当前没有真实旋转雷达的逐束运动畸变补偿，没有全局重复墙身份识别，也没有实车精度验收。此功能不会发送速度。

## 验证

- 区域选择测试：厚墙内部与双面保留、多面墙共选、弯曲墙保留、端点膨胀、范围外排除、空白/未知地图、旋转地图、保留缺口。
- 定位测试：手画线偏移不改变地图表面目标、从厚墙不同侧匹配到对应表面、平行墙不可观方向、原有特征匹配回归。
- ROS Kinetic 联调：预览、保存加载、旧数据迁移、删除撤销、保存失败不覆盖、组切换、定位 TF、特征缺失超时与地图变化失效。
