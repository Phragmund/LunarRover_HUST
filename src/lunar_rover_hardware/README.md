# lunar_rover_hardware

下位机固件与硬件部署资源目录，不属于 ROS 2 算法工作区的构建目标。

| 子目录 | 内容 |
| --- | --- |
| `Lunarover_0817/Lunarover_f407/` | STM32F407 固件工程、CubeMX 配置、FreeRTOS 与 USB CDC 代码，以及 MDK-ARM 工程文件。 |
| `ubuntu_upper_dev_handoff_20260820/` | 上位机联调的厂商 SDK、CLI/网页调试工具和部署参考。 |

固件版本、硬件接线、SDK 与设备权限必须随实车部署单独记录。ROS 层仅依赖 [`../../docs/lower_controller_contract.md`](../../docs/lower_controller_contract.md) 声明的消息和安全语义，不依赖本目录的内部文件布局。
