# Qmini Viser 界面

`viser_app.py` 是当前四轴 Qmini 的独立可视化入口。它读取
`description/qmini_arm.urdf`，提供关节滑条、末端位置 IK 和四轴反馈回放；
实机目标只通过当前仓库的 `ArmController.moveJ()` 发送。

## 安装和运行

```bash
python3 -m pip install -r requirements-viz.txt

# 只打开仿真界面，不导入串口
python3 viser_app.py --sim --mode viewer
python3 viser_app.py --sim --mode ik

# 真实机械臂：命令行授权 + 浏览器中再次勾选“启用实机驱动”
python3 viser_app.py --enable-hardware --device /dev/ttyUSB0 --mode viewer
python3 viser_app.py --enable-hardware --device /dev/ttyUSB0 --mode ik

# 只读反馈回放也需要显式授权串口
python3 viser_app.py --enable-hardware --device /dev/ttyUSB0 --mode replay
```

默认地址是 `http://127.0.0.1:8080`，可用 `--host`、`--port` 修改。当前控制器
没有自动串口发现，所以实机模式必须明确提供 `--device`/`--serial`。

### 硬件边界

没有 `--enable-hardware` 时，程序不会导入 `motor_driver` 或打开串口；连接串口也
不会自动移动机械臂。浏览器启用实机后先读取并同步当前四轴角度，再以同一姿态调用一次
`ArmController.moveJ()` 建立重力补偿保持；后续滑条或 IK 目标仍只会调用这个接口，
不会跳到仿真中位姿。退出程序时调用当前控制器的 `disable()` 和 `close()`。

离线 FK/IK、界面启动和假控制器测试不等于真实机械臂运动验证。首次实机操作仍需
确认机械零位、方向、限位、低速/低增益和可用的急停方式。
