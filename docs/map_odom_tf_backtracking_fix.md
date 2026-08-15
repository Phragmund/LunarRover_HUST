# map->odom TF 回溯卡顿问题记录

日期：2026-08-15
适用版本：`996c8c7`（master）
现象版本：`4afd746`、`ffacddc` 等未包含 `stamp_mode` 缓存的版本

## 1. 现象

RViz 中观察车辆移动时，车模会频繁卡顿：大约每秒内，车辆模型会突然被“拉回”
上一秒的位置，类似时间回溯，然后再跳回当前位置。该现象只在 RViz 视觉上明显，
其他定位/建图功能未见明显异常。

## 2. 定位过程

RViz 固定帧为 `map`，车模位置由两条 TF 合成：

```text
map -> odom        (map_odom_tf_bridge 发布)
odom -> base_link  (composite_wheel_imu_odometry 发布)
```

对 `/tf` 做了一段时间采样后得到：

| TF | 时间戳年龄 | 结论 |
| --- | --- | --- |
| `odom->base_link` | 均值约 1.6 ms，最大 72 ms | 正常 |
| `map->odom` | 均值约 687 ms，最大约 1997 ms，且 stamp 经常长时间不变 | 异常，旧时间信息来源 |

同时，`map_odom_tf_bridge` 日志中频繁出现：

```text
Skipping map->odom: LIO odometry age 2.0xx s
```

说明 bridge 内部的 `latest_lio` 经常已经滞后约 2 秒。

但独立订阅 `/localization/lio_odometry` 时，话题本身是新鲜的（age 约 1 ms、
频率约 8 Hz），说明 FAST-LIO2 发布端正常，问题出在 `map_odom_tf_bridge`
自己的订阅被饿死。

## 3. 根因

`map_odom_tf_bridge` 是单线程 executor，同时承担：

- 50 Hz 的 `publish_transform` 定时器；
- 约 150 Hz 的 `/localization/composite_odometry` 订阅；
- `TransformListener` 对 `/tf` 的订阅（包括自己发布的 TF 流量）；
- 约 8 Hz 的 `/localization/lio_odometry` 订阅。

其中 50 Hz 定时器每次都会重算并发布 `map->odom`。在 `7f1415d` 之后，
TF 的 stamp 又使用 LIO 数据时间戳：

```python
transform.header.stamp = self.latest_lio.header.stamp
```

于是形成恶性循环：

1. 50 Hz 定时器 + 高频订阅饿死了 8 Hz 的 LIO 订阅；
2. `latest_lio` 滞后 1~2 秒；
3. bridge 仍以旧 LIO 数据时间戳发布 `map->odom`；
4. RViz 用“新鲜的 `odom->base_link` + 陈旧的 `map->odom`”合成车模位置，
   每次 LIO 更新时车模位置跳变，表现为回溯卡顿。

配置中 `max_lio_age: 2.0` 放大了陈旧容忍度，使旧数据可以持续发布到 TF。

## 4. 解决方案

`996c8c7` 中已在 `map_odom_tf_bridge` 上完成修复，核心是两点：

### 4.1 缓存放行：map->odom 修正量只在收到新 LIO 时重算

新增缓存字段：

```python
self.cached_lio_stamp = None
self.cached_transform_stamp = None
self.cached_map_odom_t = None
self.cached_map_odom_q = None
```

只有当 LIO 位姿更新时，才重新计算一次 `map->odom` 修正量并缓存。
50 Hz 定时器不再用“旧 LIO + 新 local odom”反复重算，避免车模位置在
两次 LIO 更新之间被错误抵消或跳变。

### 4.2 支持 stamp_mode，默认用当前时间发布

新增参数：

```yaml
stamp_mode: now
```

`publish_transform` 中按模式选择 TF 时间戳：

```python
transform.header.stamp = (
    now_stamp if self.stamp_mode == "now" else self.cached_transform_stamp
)
```

默认 `now` 时，TF 使用当前仿真时间发布缓存的修正量，RViz 和 Nav2 可以用
高频 `odom->base_link` 平滑合成车模位置，不会再用旧 LIO 时间戳回溯。
保留 `data` 模式用于需要数据时间戳对齐的调试场景。

## 5. 验证结果

在 `996c8c7` 上重新构建并运行推荐链路后：

| 检查项 | 修复前 | 修复后 |
| --- | --- | --- |
| `map->odom` TF 时间戳年龄均值 | 约 687 ms | 约 0.5 ms |
| `map->odom` TF 时间戳年龄最大 | 约 1997 ms | 109 ms |
| late > 50 ms 的 TF 数量 | 大量 | 极少（3/711） |
| `Skipping map->odom` 日志 | 频繁 | 0 条 |
| `/localization/lio_odometry` | 约 8 Hz | 约 7 Hz |
| `/terrain/traversability` | 正常 | 正常 |
| `map -> base_link` TF | 正常 | 正常 |

## 6. 相关文件

- `src/lunar_rover_localization/scripts/map_odom_tf_bridge.py`
- `src/lunar_rover_localization/config/map_odom_tf_bridge.yaml`

## 7. 旧版最小修复参考

如果需要在未包含上述修复的旧版本（如 `ffacddc`）上快速修复，可采用更小的
改动：移除 50 Hz 定时器，改为在 `on_lio` 中直接调用 `publish_transform`，
并把 `max_lio_age` / `max_local_age` 调回 0.75。该补丁参考：
`/tmp/map_odom_tf_bridge_ffacddc_fix.patch`。
