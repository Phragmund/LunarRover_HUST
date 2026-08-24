# Web 调试工具使用说明

## 1. 交付内容

`tools/web/` 目录内包含：

- `rover_web_controller.html`
  当前主用统一调试页，集成 3508、舵轮、机械臂
- `m3508_web_controller.html`
  早期 3508 独立调试页
- `m3508_rx_monitor.html`
  3508/CDC 回读监视页
- `run_web_controller.bat`
  Windows 启动脚本
- `run_rx_monitor.bat`
  Windows 启动脚本
- `start_web_controller.sh`
  Ubuntu 启动脚本
- `start_rx_monitor.sh`
  Ubuntu 启动脚本

## 2. Ubuntu 启动方式

进入 `tools/web/` 后执行：

```bash
chmod +x start_web_controller.sh start_rx_monitor.sh
./start_web_controller.sh
```

默认地址：

- 统一调试页：`http://localhost:8088/rover_web_controller.html`
- 3508 独立页：`http://localhost:8088/m3508_web_controller.html`

回读监视页：

```bash
./start_rx_monitor.sh
```

地址：

- `http://localhost:8089/m3508_rx_monitor.html`

## 3. 浏览器要求

网页使用 `Web Serial`。

建议：

- Ubuntu 使用 `Chromium` 或 `Google Chrome`
- 页面中点击“连接 H7 USB”后，手动选择 H7 的虚拟串口

注意：

- 同一时间只能有一个程序占用串口
- 浏览器打开串口后，Python 串口脚本就不能再同时打开
- 如果要让 CLI 接管串口，先在网页里点断开

## 4. 当前统一调试页功能

### 4.1 3508

- 快捷前进/后退/转向
- 四电机自定义速度或位置命令
- 原始 `$M3508:*` 命令发送
- 在线反馈显示

### 4.2 舵轮

- 四路角度模式
- 四路速度模式
- `$ZERO!` / `$STOP!`
- 四路板 RAW 透传
- `$STD` 反馈显示

### 4.3 机械臂

- 单关节控制
- 多关节联动
- IK 解算与键盘控制
- 动作设计（时间轴编排动作、播放 / 逆行回起点、导出动作文件）
- RAW 指令透传
- Home 姿态回位

动作设计模式的详细用法、导出动作文件格式与上位机回放教程见 [`04_action_design_guide.md`](04_action_design_guide.md)。

## 5. 当前网页已修复的问题

截至 2026-08-20，统一页已包含这些关键修复：

- `IK 初始位` 不再只让机械臂回位，前端缓存姿态和示意图也会同步归位
- 机械臂应答帧已按单条 `$ARM:RESP,...!` 形式输出，降低 USB 文本交叉导致的乱码
- 轮序标签已经与实机编号统一：
  - `1 右后`
  - `2 左后`
  - `3 右前`
  - `4 左前`

## 6. Ubuntu 上的建议使用顺序

1. 先只开 `rover_web_controller.html`
2. 先验证串口连接和回读是否正常
3. 再分模块测试 3508 / 舵轮 / 机械臂
4. 若需要纯观察回读，再关闭统一页、改开 `m3508_rx_monitor.html`

## 7. 已知局限

- 当前 `$STD` 遥测对舵轮并不完整，只够轻量调试
- 机械臂高频 `READ` 不是主要工作模式，连续高频读会比 `PING` 更容易触发超时或控制板回包异常
- `run_*.bat` 只适合 Windows，Ubuntu 请用 `.sh` 脚本或手工 `python3 -m http.server`

