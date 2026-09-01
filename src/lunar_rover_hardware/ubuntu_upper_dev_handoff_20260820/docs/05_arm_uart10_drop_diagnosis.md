# 机械臂 UART10 应答截断问题（$ARM:RESP,#!）分析与修复建议

> 目标读者：下位机（H7 固件）负责人
> 关联：上位机动作设计播放时机械臂丢包问题
> 日期：2026-08-25

## 1. 现象

上位机（网页调试台）在播放机械臂动作、周期读取舵机位置时，偶发收到一条损坏的应答帧：

    [15:42:58] TX ×2 $ARM:MOVE,1,1833,800!
    [15:42:58] RX $ARM:RESP,#!
    [15:42:59] TX ×2 $ARM:MOVE,4,944,808!
    [15:42:59] RX $ARM:OK,MOVE,4,944,808!

正常的位置应答应是 $ARM:RESP,#001P1833!（# + 3 位舵机号 + P + 4 位脉冲），而这里只剩 $ARM:RESP,#! —— 舵机号和脉冲全丢了，只剩一个 #。

表现为：偶发，在“机械臂动作 + 周期 READ 轮询”同时进行时概率升高；某个关节的反馈拿不到，上位机据此判定该关节丢包。

## 2. 消息链路

    PC(网页) --USB CDC--> H7 --UART10(115200)--> 机械臂舵机控制板

位置读取流程：

1. 上位机发 $ARM:READ,1!
2. H7 解析后，往 UART10 发原始舵机指令 #001PRAD!
3. 舵机板回 #001P1833!
4. H7 把应答封装成 $ARM:RESP,#001P1833! 经 USB 发回 PC

## 3. 关键代码位置

下位机工程：CtrBoard-H7_ALL（0821）

| 文件 | 函数/位置 | 作用 |
| --- | --- | --- |
| App/arm_servo_task.c | handle_read() | 处理 $ARM:READ |
| App/arm_servo_task.c | uart_read_response() | 阻塞逐字节读 UART10 应答 |
| App/arm_servo_task.c | arm_send_response() | 组 $ARM:RESP,... 帧 |
| Core/Src/usart.c | MX_USART10_UART_Init() | UART10 初始化，使能 USART10_IRQn |
| Core/Src/stm32h7xx_it.c | USART10_IRQHandler() | 调 HAL_UART_IRQHandler(&huart10) |
| Core/Src/freertos.c | 任务创建 | ArmServoTask = Normal；M3508Task / ImuTask = High |

核心读应答代码（uart_read_response）：

    while (超时未到)
    {
        if (HAL_UART_Receive(&huart10, &ch, 1U, 5U) == HAL_OK)  // 一次只读 1 字节，阻塞
        {
            // 存字节
            if (ch == '!') break;   // 以 ! 结束
        }
    }

## 4. 现象产生过程（推导）

$ARM:RESP,#! 说明 uart_read_response() 只读到了第一个 #，后面的 001P1833! 全部丢失。

UART10 是 115200 波特率、8N1，每个字节间隔约 86.8 微秒。而 uart_read_response 是“逐字节 + 阻塞 HAL_UART_Receive”：读一个字节返回一次、再重新进函数读下一个字节，中间有函数调用和状态机开销；更关键的是，机械臂任务优先级是 Normal，会被 High 优先级的 M3508Task、ImuTask 抢占，也会被 USB CDC、FDCAN、UART7 等中断打断。

一旦在两个字节之间被拖住超过约 86.8 微秒，下一个字节到达时 RDR 还满着，就会触发 UART Overrun Error（ORE，溢出错误）：该字节被硬件丢弃。ORE 一旦发生，后续字节也可能连续丢失，最终只回一个 #。

## 5. 具体分析：为什么是“竞争”导致丢字节

两个因素叠加：

1. 逐字节阻塞读本身对时序敏感：CPU 空闲、不被抢占时能跟上 86.8 微秒/字节；一旦被抢占，就漏字节。

2. 阻塞读与错误中断混用：HAL_UART_Init 默认打开 EIE（错误中断：帧错/噪声/溢出），MX_USART10_UART_Init 又使能了 USART10_IRQn。溢出发生时由中断去清 ORE 标志，而“任务里的阻塞读”和“中断里的错误处理”共同操作同一个 UART 外设，字节流很容易被搅乱/截断。

补充（已排除）：App/fun_test_task.c 里有一句 HAL_UART_Receive_IT(&huart10, ...)（中断方式占用 UART10），但 freertos.c 里 FunTest 任务并未创建（FunTestHandle = NULL），属死代码，不是本问题原因。

## 6. 建议修复方向（按优先级）

方案 A（最彻底，推荐）：UART10 改用“中断/DMA 接收 + 环形缓冲区”，机械臂任务从缓冲区按 ! 组帧，不再逐字节阻塞读。ORE 由 HAL 中断路径统一处理，彻底消除“阻塞读 + 错误中断”的竞争。

方案 B（最小改动，可先验证）：

- 在 uart_read_response() 读之前显式清 ORE（__HAL_UART_CLEAR_OREFLAG(&huart10)）；
- 读的过程中若发现字节丢失（超时/ORE），flush 后重试一次这条 READ；
- 或把 ArmServoTask 优先级提到 High，降低被抢占概率。

方案 C（辅助定位）：先加一个 ORE 计数器，读 huart10.ErrorCode 里 ORE 标志是否置位，跑一段确认是不是溢出导致，再决定改 A 还是 B。

## 7. 需要下位机确认的点

- 实测 huart10.ErrorCode 中 ORE / FE / NE 是否置位，确认是溢出/帧错，还是舵机板本身回了个空 #。
- 舵机板在繁忙时是否会主动回一个 # 或 #!（排除舵机板固件问题）。
