# smartcar_navigation

单点规划与执行。入口 scripts/single_goal_nav.py；几何和全局规划 ackermann_core.py；局部预测 local_planner.py/actuator_model.py；跟踪 path_tracking.py；配置 nav_config.py。只消费驱动与定位输出，不启动 Gazebo、雷达、建图或定位。
