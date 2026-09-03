# 下位机部署资源

该目录保留 H7 上位机联调所需的厂商 SDK、网页调试工具与部署参考。它不是 ROS 公共接口的一部分，也不应被上层算法直接依赖。

底盘 ROS 接口和安全规则以仓库 [`docs/lower_controller_contract.md`](../../../docs/lower_controller_contract.md) 为准。部署时须单独记录固件版本、SDK 来源、设备权限和硬件配置。

- `tools/cli/`：M3508 USB/CAN 命令行诊断工具；
- `tools/web/`：浏览器串口调试页及启动脚本；
- `docs/`：厂商工具和串口诊断参考。

这些工具仅用于设备部署和诊断，不得在自主导航、任务规划或竞赛演示主链路中充当控制接口。
