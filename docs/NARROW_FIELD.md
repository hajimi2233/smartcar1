# 小地图几何

`data/maps/sim_field_narrow` 是 `sim.sh start` 默认使用的窄场景。Gazebo 世界和 RViz 的 `map.yaml`、`map.pgm` 由同一生成器产生：

```bash
python3 ~/smartcar1/scripts/generate_narrow_field.py
```

当前几何参数：

- 通道墙长度：3.6 m，端点在 y=±1.8 m；
- 通道墙厚度：0.30 m；
- 两墙内表面净宽：0.90 m（墙中心距 1.20 m）；
- 通道端部到上下外墙内表面的净调整空间：各 1.50 m；
- 上下外墙中心：y=±3.34 m，厚度 0.08 m。

通道中心线为 x=-2.4、-1.2、0、1.2、2.4 m。地图栅格仍为 0.05 m，世界坐标原点不变。旧地图上的点位和低代价线不能直接复用：通道中心间距已经从 0.98 m 改为 1.20 m，必须重新标定点和重新画线。

生成器只更新地图与世界文件，不覆盖用户已经画好的低代价线。启动窄场景：

```bash
cd ~/smartcar1
bash scripts/sim.sh build
bash scripts/sim.sh start narrow
bash scripts/sim.sh rviz
```

定位和导航使用同一个 `data/maps/sim_field_narrow/map.yaml`。
