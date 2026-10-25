# 不规则摄像头预览停顿

已部署；未启动摄像头。自行运行 `~/camera` 或 `~/camera real` 即可使用。

替代固定每 4 秒停顿的版本。现在普通停顿间隔随机约 2.4～6.8 秒，时长多数为约 0.16～0.56 秒，少数约 0.64～1 秒；部分停顿恢复后，会在 0.2～0.6 秒内再顿一下，形成短暂聚集。实际观感受 5 FPS 的刷新间隔影响，屏幕刷新间隔可比设定保持时长多一帧左右。

这是人为模拟的显示效果，启动日志明确标记 `[SIMULATED DISPLAY JITTER]`。采集和识别仍按原循环执行，没有增加阻塞等待或帧队列；恢复后直接显示最新图像。停顿期间 GUI 继续处理按键，q/Esc 可退出。无窗口模式、静态图片、保存结果和识别事件不添加该效果。分类与框取类语法树核对相同，导航和巡检任务代码未修改。

```bash
~/camera
# 关闭效果
~/camera --display-hold-seconds 0
# 更轻微：降低典型停顿时长并拉大间隔
~/camera --display-hold-seconds 0.25 --display-hold-every 6
# 使用可重复的随机序列便于调试
~/camera --display-seed 17
```

车端 Python2 模拟采集/GUI 验证通过：160 帧处理时序一致；显示刷新由 160 次降为 146 次，该样本刷新间隔为 0.2、0.6、0.8 秒；固定种子可复现、不同种子显示行为不同；headless 不受影响、停顿期间 q 可退出；另外以模拟 50Hz 刷新验证保持有限且能够恢复。这些检查未打开实际相机或窗口，实际观感由用户启动后确认。

备份：`/home/smartcar/vision_v5/backup_display_jitter_20261017_133941/overhead_pic_detector_v5.py`。退出摄像头后，将备份覆盖回 `/home/smartcar/vision_v5/overhead_pic_detector_v5.py` 可恢复固定停顿版本。本地 `before/`、`candidate/` 及 `results/` 保存改动前后代码与验证、部署记录。
