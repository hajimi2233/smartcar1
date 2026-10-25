# 实车默认定位恢复（2026-09-30）

默认算法：laser_scan_matcher + AMCL。恢复仓库当前默认链路；历史自编节点仅保留为实验备份，不与本入口同时运行。

每个终端先 source ~/smartcar_2026_ws/scripts/source_env.sh。
先启动雷达：roslaunch smartcar_mapping lidar.launch。
再启动定位：roslaunch smartcar_localization localization_real.launch。
该入口会在没有 master 时自动启动 master，不启动底盘，不包含导航器。

当前实车入口 base_frame 默认 laser：map -> odom -> laser，先验证雷达定位。
算法与仓库默认相同，车体参考点暂时不同。测量雷达外参后，再提供唯一的
base_footprint -> laser 静态 TF，并传 base_frame:=base_footprint，恢复车体定位链。
不能同时运行 custom_localization.launch、Cartographer 或另一个里程计 TF 发布者。

参数：config/amcl.yaml，复制自仓库 config/localization.yaml。
AMCL 用 likelihood_field，静止时并非每帧更新；update_min_d=0.03 m、update_min_a=0.05 rad。
RViz Fixed Frame=map，Map=/map，LaserScan=/scan，PoseWithCovariance=/amcl_pose。
2D Pose Estimate 当前标雷达中心和扫描坐标朝向；旧 /localization/preview_scan 不再由默认方案发布。

部署启动检查与硬件运动精度验收是不同步骤。必须完成静止、直行、转弯和返回起点检查后才考虑自动导航。
