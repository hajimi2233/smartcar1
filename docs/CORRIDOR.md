# 通道内外判断（画框 + 定位参考点）

使用现有 RViz **Draw Region** 四角画框工具定义通道范围。判断点是导航配置 `tuning.base_frame`（默认 `base_footprint`）在 `map` 中的 TF 位置，不做前轴/后轴偏移，不按车身轮廓判断。点在框内或边界上均为通道内；框外为通道外。车头角度不影响结果。

当前复用一个凸四边形，按顺时针或逆时针点击四角，可以画旋转矩形。不支持多个独立通道的编号识别；若框住多个通道及其隔墙，框内的隔墙和空隙也会被算作通道内，因此应按实际需要选取范围。此节点只提供位置状态，不会修改导航策略或发送速度。

## 仿真操作

源码改动后先 `bash scripts/sim.sh build`，再启动仿真和地图定位，具体见 REPRODUCE.md。若旧容器正在运行，构建后需重新 `bash scripts/sim.sh start` 更新容器，并重启定位及相关工具。

另开终端启动区域编辑器和判断节点：

```bash
cd ~/smartcar1
bash scripts/sim.sh corridor
```

默认区域文件为容器项目 `data/maps/slam_current/external_region.json`，映射到本机同名数据目录。可通过 `region_file:=/home/hajimi/smartcar/data/maps/你的地图/corridor.json` 指定单独文件；现有区域文件若与地图匹配，会直接加载，请确认其范围确实代表想判断的通道。

再开终端与 RViz：

```bash
cd ~/smartcar1
bash scripts/sim.sh rviz
# 另开终端：
bash scripts/sim.sh region begin
```

RViz 的 Fixed Frame 使用 `map`，选择工具栏 **Draw Region**（快捷键 B），沿通道边界依次点四个角，然后保存：

```bash
bash scripts/sim.sh region save
bash scripts/sim.sh corridor-state
```

`region undo` 撤销最后一个草稿点，`region cancel` 取消草稿，`region clear` 删除已保存区域。草稿不影响当前判定，只有保存后生效。RViz 会显示区域边框和 `Corridor: INSIDE/OUTSIDE/UNKNOWN` 文字。

实车已 source ROS 和 catkin 工作空间后，把上述 `scripts/sim.sh` 换成 `scripts/robot.sh`；源码更新需重新 catkin 编译。节点读取当前仿真/实车导航配置中的 TF 帧名与定位超时参数。

## 输出接口

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/corridor/state` | `std_msgs/String` | `INSIDE` 通道内、`OUTSIDE` 通道外、`UNKNOWN` 无法判断 |
| `/corridor/reason` | `std_msgs/String` | 使用的参考坐标系或无法判断的原因 |
| `/corridor/marker` | `visualization_msgs/Marker` | RViz 状态文字 |

更新频率 10 Hz。没有有效区域、区域已清除、地图指纹不匹配、TF 缺失或过期时发布 `UNKNOWN`。TF 超时沿用 `tuning.pose_timeout`，默认 0.4 秒。区域固定在 `map` 坐标系，当前启动会拒绝其他 `map_frame` 配置。区域绑定完整地图指纹，换地图必须重新画框；应配合已保存地图的定位使用，不用于地图持续变化的建图阶段。

判断在边界上不做时间滤波，定位抖动可能引起内外切换。上层订阅者应检测消息新鲜度，节点退出后不能继续使用最后一次状态来控制车辆。
