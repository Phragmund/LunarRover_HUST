# lunar_rover_msgs

跨包公共消息定义：

- `ChassisState.msg`：底盘模式、故障位、急停、使能与命令看门狗状态；
- `ManualControl.msg`：人工输入的杆量、控制按键和控制权请求；
- `TargetObservation.msg`：目标类别、置信度、位姿、尺寸和表面属性。

消息字段的物理语义与使用限制见 [`../../docs/interface_contract.md`](../../docs/interface_contract.md)。修改消息定义属于破坏性接口变更，必须同步更新所有发布者、订阅者、测试与版本说明。

`msg/` 保存接口源文件；`CMakeLists.txt` 与 `package.xml` 负责 rosidl 消息生成。业务逻辑不得放入本包。
