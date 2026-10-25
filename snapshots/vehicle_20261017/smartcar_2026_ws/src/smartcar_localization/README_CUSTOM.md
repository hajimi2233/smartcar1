# 自编地图匹配定位：实车第一阶段

来源：Git 提交 2cf7cb6 的 docker/overlay/scan_match_localizer.py。
算法直接将激光匹配到保存的地图；不依赖 laser_scan_matcher 或 AMCL。

当前默认输出雷达位置：map -> odom（恒等）-> laser。
这不是连续局部里程计，后续导航使用前需统一坐标接口。
不要同时运行 Cartographer、AMCL 或另一个相同 TF 的发布者。

## 启动
每个终端先 source ~/smartcar_2026_ws/scripts/source_env.sh。
先用现有 smartcar_mapping/lidar.launch 启动雷达。
然后运行：

```bash
roslaunch smartcar_localization custom_localization.launch map_file:=/home/smartcar/smartcar_2026_ws/data/maps/map_20260928_164936/map.yaml
```

在车载桌面启动 RViz，Fixed Frame=map，显示 /map、/scan 和 /lidar_pose。
使用 2D Pose Estimate 标出雷达中心的位置及扫描坐标系朝向。
节点不控制底盘。先静止检查，再用已验证的人工控制方式低速移动测试。

## 参数与外参
config/scan_match.yaml 保存搜索窗口、匹配分数、采样和超时等参数。
launch 的 laser_x、laser_y、laser_yaw 分别为雷达相对 pose_frame 的米/米/弧度偏移。
默认 pose_frame=laser、偏移为零，仅验证雷达定位。
切换到后轴中心参考点时，设置 pose_frame=base_link，填入实测外参，
并单独发布唯一的 base_link -> laser 静态 TF；不得使用文档估算值充当实测值。

## 验收边界
超过 scan_timeout 没有成功匹配时不再刷新 TF；历史 TF 仍可能保留在缓冲区。
消费者必须检查时间戳。/lidar_pose 协方差是旧算法固定值，不是可信度估计。
只支持零旋转的地图 origin。未完成实车运动精度及运行耗时验收。
重复通道场景可能发生错误匹配；应测试直行、转弯和返回起点。
