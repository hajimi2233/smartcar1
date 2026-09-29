# 实车驱动接入约定

具体底盘协议、雷达型号、串口和波特率尚未提供。本工程不猜测这些值。先让厂商驱动独立运行，再用两个 launch 文件接入 `drivers-real`：一个启动底盘及转换桥，一个启动雷达及其静态外参。

## 雷达

- 输出 `sensor_msgs/LaserScan` 到 `/scan`；header.stamp 是实际采样时间，header.frame_id 是雷达 TF 名称。
- 提供从 `base_footprint` 到雷达 frame 的实测平移和旋转 TF，且只能有一个发布者。
- 实车没有 `/sim/scan` 或 laser_noise 节点；需要的去畸变/滤波应在驱动侧明确加入。

## 底盘

默认实车配置消费以下接口，可在 `config/navigation-real.yaml` 修改话题和模型参数路径：

| 接口 | 约定 |
|---|---|
| `/vehicle/cmd_vel` | 输入 Twist；linear.x 是后轴中心纵向速度 m/s，angular.z 是车体角速度 rad/s |
| `/vehicle/joint_states` | 输出实际轮速与转向反馈 JointState，有正确时间戳 |
| `/vehicle/actuator_model` | ROS 参数字典，提供执行器模型和尺寸 |

JointState 的 name 包含 `front_left_steer_joint`、`front_right_steer_joint`、`rear_left_wheel_joint`、`rear_right_wheel_joint`。前两个 position 为实际左右前轮转角（rad），后两个 velocity 为实际后轮角速度（rad/s），数组与名称一一对应。只有中心转角或车速反馈时，桥接节点负责换算。不能用发出的转向指令冒充实际反馈。

执行器模型字段为 `version: 1`、`wheelbase`、`front_track`、`wheel_radius`、`acceleration`、`braking`、`steer_rate`。尺寸单位米，加减速度 m/s²，等效中心转角速率 rad/s。前三项必须与导航配置一致。数值应根据车上实测填写，仿真参数不等于已标定的实车参数。

如果厂商驱动接收速度和转角，桥接层将角速度换算为等效中心转角；零速时不能直接除以速度。桥接层应明确低速/零速转向行为。驱动还需负责通信丢失时停车、控制权切换和真实急停，本次目录重构不实现这些硬件功能。

## 时钟与定位参考点

实车 `use_sim_time=false`，不依赖 Gazebo `/clock` 或真值。底盘定位原点若在车体中心、前轴或其他位置，配置 `tuning.base_reference` 及偏移；车身坐标轴遵守前方 +x、左方 +y。

默认定位前端使用激光里程计发布 `odom → base_footprint`。如果使用底盘/融合里程计代替，启动建图、定位时都加 `scan_odometry:=false`，保留唯一发布者。TF 来源是部署决策，不通过重命名话题自动解决。

## 第一层验收

1. 厂商工具能可靠启动、停车、前进、后退和转向，方向与单位正确。
2. 雷达角度、距离、外参正确，车静止时扫描稳定。
3. 时间戳持续更新；车辆参考点和尺寸完成测量。
4. `robot.sh check drivers` 能收到标准接口。该检查不主动发车速，也不证明运动精度。

只有完成这些接口转换，才具备复用现有导航的条件；仅设置 `/vehicle/...` 话题名并不等于底盘已适配。
