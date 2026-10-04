# 缩小上下回转区的仿真场地

新场地在 `data/maps/sim_field_narrow`，原场地保持可用。
上外墙中心从 y=4.0 移到 y=3.2，下外墙从 y=-4.0 移到 y=-3.0；左右外墙长度同步从 8.0 改为 6.2 m，中心改为 y=0.1。通道墙和通道内目标位置不变。

通道端点在 ±1.8，外墙厚 0.08，因此上、下净回转空间分别 1.36、1.16 m。
地图是从新 SDF 的雷达高度碰撞盒栅格化生成的 5 cm 栅格地图，坐标为世界坐标，无旧 SLAM 地图配准偏移。并不是在改场地后仍使用旧 slam_current 地图。

生成命令：

```bash
python3 ~/smartcar1/scripts/generate_narrow_field.py
```

生成器会重写该场景的地图及候选低代价线；重新画线后不要再次运行生成器，除非先备份。

启动新场地（停止现有定位/导航终端后执行；会重建仿真容器）：

```bash
cd ~/smartcar1
bash scripts/sim.sh build
docker compose -p smartcar-layered -f docker-compose.yml -f docker-compose.narrow.yml up -d --force-recreate sim
```

新终端启动定位：

```bash
bash ~/smartcar1/scripts/sim.sh localization /home/hajimi/smartcar/data/maps/sim_field_narrow/map.yaml
```

RViz 中重新设置初始位姿，然后新终端启动导航：

```bash
bash ~/smartcar1/scripts/sim.sh single low_cost_lines_file:=/home/hajimi/smartcar/data/maps/sim_field_narrow/low_cost_lines.json
```

```bash
bash ~/smartcar1/scripts/sim.sh rviz
```

候选低代价线为 y=2.48 和 y=-2.38。原低代价线已过于靠近新外墙，不能直接沿用；新线是测试候选，不声称是最优画法。新地图和新线单独保存，不覆盖原地图对应的画线记录。当前场地使用地图匹配定位，不启用旧场地的优先墙/区域记录。

恢复原场地（随后使用原地图重新启动定位和导航）：

```bash
cd ~/smartcar1
docker compose -p smartcar-layered -f docker-compose.yml up -d --force-recreate sim
```
