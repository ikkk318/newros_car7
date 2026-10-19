# real/ 覆盖层 —— 实车专属参数

默认**空目录**：不传 `param_profile:=real` 时它完全不参与。
接真实硬件后，把与 `default/` 不同的参数写进这里的**同名文件**，
启动时传 `param_profile:=real` 就会覆盖。

```bash
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true param_profile:=real
```

只写要改的键，不用整份复制——加载器做的是深合并（`agv_common.config_loader`）。
