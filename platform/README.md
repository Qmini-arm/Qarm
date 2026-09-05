# Qarm Control Platform

Qarm 的第一版网页控制台，参考机器人控制平台说明文档 v1.1.1 的信息架构，面向六轴 GO-M8010-6 机械臂。

## 本地运行

```bash
cd platform
npm install
npm run dev
```

浏览器打开 <http://127.0.0.1:5173>。

## Viser 3D

平台右侧 3D 区可以切换到 Viser。先在另一个终端启动现有的 Qarm Viser：

```bash
cd ..
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qmini-motion viz --host 127.0.0.1 --port 8080
```

默认 iframe 地址为 `http://127.0.0.1:8080`。如果 Viser 在另一台机器或端口：

```bash
VITE_VISER_URL=http://192.168.10.102:8080 npm run dev
```

平台当前使用模拟遥测让 UI 在没有硬件时可交互；串口和真实动作仍由 Qarm 的 C++ 控制器与 Python 规划器负责，前端没有直接串口权限。

## 在开发板上运行

平台服务可以和网页静态文件一起部署到开发板；浏览器只需访问开发板地址，电脑上不需要
启动 Node/Vite：

```bash
# 在开发电脑执行（默认 HwHiAiUser@192.168.10.102）
./platform/deploy_board.sh --no-start

# SSH 到板端启动。QARM_HARDWARE=1 只打开六轴反馈读取；未安装真实执行器时，
# 使能、重力补偿和 MOVEJ 会明确返回 501，不会伪报成功。
ssh HwHiAiUser@192.168.10.102 \
  'cd ~/qarm-platform && QARM_HARDWARE=1 QARM_PLATFORM_PORT=8090 \
   ./platform/run_server.sh'
```

然后打开 <http://192.168.10.102:8090/>。`deploy_board.sh` 会在本地构建 React bundle，
复制 `platform/dist`、控制服务和关节映射；板端只需要 Python 3。可用环境变量覆盖目标：
`QARM_BOARD_HOST`、`QARM_BOARD_USER`、`QARM_BOARD_ROOT`。如需在板端运行 Viser，另行启动
`qmini-motion viz --host 0.0.0.0 --port 8080`，并在构建时设置
`VITE_VISER_URL=http://192.168.10.102:8080`。
