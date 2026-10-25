# loc 回退后的启动修复

已确认启动报错为 `ValueError: Invalid corridor_weighting keys`。原因是 ROS 参数服务器残留固定墙版本的 `fixed_walls_only=true`，恢复后的通道加权版本不接受此键。

修复仅在 `~/loc` 原有运行节点检查之后、启动 roslaunch 之前清除该退役参数。程序文件、地图、其他参数不在本补丁修改范围内。

车端备份：`/home/smartcar/loc.before_stale_mode_fix_20261017_202145`。有时限的实际启动验证通过，检查后已关闭定位。未发送运动指令。

本问题说明仅检查文件、测试与导入不足以验证回退后的 ROS 运行状态；应包含残留参数的启动验证。
