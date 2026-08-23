#include "steer_motor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "usb_debug.h"
#include "usbd_cdc_if.h"

#define STEER_MOTOR_TASK_PERIOD_MS 20U

/* 1=每100ms发JustFloat二进制帧(VOFA+画曲线, 替换$STD), 0=发$STD文本遥测 */
#define STEER_VOFA_JUSTFLOAT 0U

/* 命令密度控制: 板子处理不过来会丢命令, 反馈反而变慢.
   编码器回读和速度下发都降频, 避免一秒上百条把板子灌满. */
#define STEER_ENCODER_READ_DIV  3U  /* 每 3 圈(约60ms)请求一次编码器回读 -> ~16Hz 反馈 */
#define STEER_SPEED_SEND_DIV    2U  /* 每 2 圈(约40ms)下发一次速度命令 -> 25Hz, 降低板子命令负载 */

/* 锁死通道掩码: 置 1 的通道强制 0 速度(板子速度环锁定), 只有置 0 的通道响应串口命令.
   位序 = steer_motor[0..3] = 驱动板通道0..3.
   当前配置 0x0C = 锁通道2/3, 开放通道0/1. 0x00=全部开放, 0x0D=锁0/2/3放1. */
#define STEER_LOCKED_CHANNEL_MASK  0x00

/* USB CDC 命令接收缓冲(中断写入, 任务读取) */
#define CDC_RX_BUF_SIZE 128U

static float steer_angle_cmd_deg[STEER_MOTOR_NUM] = {0.0f};
static float steer_speed_cmd_norm[STEER_MOTOR_NUM] = {0.0f};
static uint8_t steer_enable = 0U;
static uint8_t steer_speed_mode = 0U;
static uint8_t steer_zero_pending = 0U;
static uint32_t steer_last_command_ms = 0U;

/* USB CDC 接收: 中断追加, 任务消费 */
static volatile uint8_t cdc_rx_buf[CDC_RX_BUF_SIZE];
static volatile uint16_t cdc_rx_len = 0U;

/* 命令帧组装(任务上下文) */
static char cmd_frame[64];
static uint16_t cmd_frame_len = 0U;

void SteerMotorTask_SetAngle(uint8_t index, float angle_deg)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    taskENTER_CRITICAL();
    steer_angle_cmd_deg[index] = angle_deg;
    steer_speed_mode = 0U;
    steer_enable = 1U;
    steer_last_command_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SteerMotorTask_SetAllAngles(const float angle_deg[STEER_MOTOR_NUM])
{
    uint8_t i;

    if (angle_deg == 0)
        return;

    taskENTER_CRITICAL();
    for (i = 0U; i < STEER_MOTOR_NUM; i++)
        steer_angle_cmd_deg[i] = angle_deg[i];

    steer_speed_mode = 0U;
    steer_enable = 1U;
    steer_last_command_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SteerMotorTask_SetSpeed(uint8_t index, float speed_norm)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    taskENTER_CRITICAL();
    steer_speed_cmd_norm[index] = speed_norm;
    steer_speed_mode = 1U;
    steer_enable = 1U;
    steer_last_command_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SteerMotorTask_SetAllSpeed(const float speed_norm[STEER_MOTOR_NUM])
{
    uint8_t i;

    if (speed_norm == 0)
        return;

    taskENTER_CRITICAL();
    for (i = 0U; i < STEER_MOTOR_NUM; i++)
        steer_speed_cmd_norm[i] = speed_norm[i];

    steer_speed_mode = 1U;
    steer_enable = 1U;
    steer_last_command_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SteerMotorTask_SetZero(uint8_t index)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    taskENTER_CRITICAL();
    steer_zero_pending |= (uint8_t)(1U << index);
    taskEXIT_CRITICAL();
}

void SteerMotorTask_SetAllZero(void)
{
    taskENTER_CRITICAL();
    steer_zero_pending = (uint8_t)((1U << STEER_MOTOR_NUM) - 1U);
    taskEXIT_CRITICAL();
}

void SteerMotorTask_Stop(void)
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        steer_angle_cmd_deg[i] = 0.0f;
        steer_speed_cmd_norm[i] = 0.0f;
    }

    steer_enable = 0U;
    taskEXIT_CRITICAL();
}

void SteerMotorTask_Enable(uint8_t enable)
{
    if (!enable)
    {
        SteerMotorTask_Stop();
        return;
    }

    taskENTER_CRITICAL();
    steer_enable = 1U;
    steer_last_command_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

/* USB CDC 收到数据(中断上下文): 只做追加, 解析在任务里 */
void SteerMotorTask_CdcRxCallback(uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (data == 0)
        return;

    for (i = 0U; i < len; i++)
    {
        if (cdc_rx_len < (CDC_RX_BUF_SIZE - 1U))
            cdc_rx_buf[cdc_rx_len++] = data[i];
    }
}

/* 命令解析: $SPD,s0,s1,s2,s3! / $ANG,a0,a1,a2,a3! / $STOP! / $ZERO! /
   $MOTOR_4CH...(透传到四路板) */
static void cmd_parse(const char *frame)
{
    float v[STEER_MOTOR_NUM];

    /* 四路板配置/读取指令: 从 USB 原样透传到 USART1, 便于手动下发 SET 等 */
    if (strncmp(frame, "$MOTOR_4CH", 10U) == 0)
    {
        SteerMotor_SendRaw(frame);
        return;
    }

    if (sscanf(frame, "$SPD,%f,%f,%f,%f!", &v[0], &v[1], &v[2], &v[3]) == 4)
    {
        SteerMotorTask_SetAllSpeed(v);
        return;
    }
    if (sscanf(frame, "$ANG,%f,%f,%f,%f!", &v[0], &v[1], &v[2], &v[3]) == 4)
    {
        SteerMotorTask_SetAllAngles(v);
        return;
    }
    if (strncmp(frame, "$STOP!", 6) == 0)
    {
        SteerMotorTask_Stop();
        return;
    }
    if (strncmp(frame, "$ZERO!", 6) == 0)
    {
        SteerMotorTask_SetAllZero();
        return;
    }
}

static void cmd_feed_byte(uint8_t c)
{
    if (c == '$')
        cmd_frame_len = 0U;

    if (cmd_frame_len < (sizeof(cmd_frame) - 1U))
        cmd_frame[cmd_frame_len++] = (char)c;

    if (c == '!')
    {
        cmd_frame[cmd_frame_len] = '\0';
        cmd_parse(cmd_frame);
        cmd_frame_len = 0U;
    }
}

void SteerMotorTask_Entry(void const *argument)
{
    uint8_t i;

    (void)argument;

    /* 等四路板上电就绪再下发初始化命令, 否则板子收不到 SET 指令 */
    osDelay(500);

    SteerMotor_Init(&huart1);
    SteerMotor_StopAll();
    osDelay(100);

    for (;;)
    {
        float angle_command[STEER_MOTOR_NUM];
        float speed_command[STEER_MOTOR_NUM];
        uint8_t enabled;
        uint8_t speed_mode;
        uint8_t zero_mask;
        static uint8_t enc_read_div_cnt = 0U;
        static uint8_t speed_send_div_cnt = 0U;

        if ((enc_read_div_cnt++ % STEER_ENCODER_READ_DIV) == 0U)
            SteerMotor_RequestEncoderTotal();
        SteerMotor_RxPoll(100U);

        /* 板子确认在线后, 补发一次 SET 配置(防止开机时板子未就绪漏掉) */
        {
            static uint8_t cfg_sent = 0U;
            if (!cfg_sent && SteerMotor_IsOnline(0))
            {
                cfg_sent = 1U;
                SteerMotor_SendRaw("$MOTOR_4CH_SET:2!");
                SteerMotor_SendRaw("$MOTOR_4CH_SET_ENCPDER_POLARITY:0!");
            }
        }

        /* 处理 USB CDC 命令 */
        if (cdc_rx_len > 0U)
        {
            uint8_t tmp[CDC_RX_BUF_SIZE];
            uint16_t tmp_len;

            taskENTER_CRITICAL();
            memcpy(tmp, (const void *)cdc_rx_buf, cdc_rx_len);
            tmp_len = cdc_rx_len;
            cdc_rx_len = 0U;
            taskEXIT_CRITICAL();

            for (i = 0U; i < tmp_len; i++)
                cmd_feed_byte(tmp[i]);
        }

        /* 读取命令状态 */
        taskENTER_CRITICAL();
        for (i = 0U; i < STEER_MOTOR_NUM; i++)
        {
            angle_command[i] = steer_angle_cmd_deg[i];
            speed_command[i] = steer_speed_cmd_norm[i];
        }
        enabled = steer_enable;
        speed_mode = steer_speed_mode;
        zero_mask = steer_zero_pending;
        steer_zero_pending = 0U;
        taskEXIT_CRITICAL();

        /* USB 主机失联后不允许最后一条位置/速度命令永久有效。上位机
           正常工作时每 100ms 重发 $ANG，超过 200ms 没有控制命令即停。 */
        if (enabled && ((HAL_GetTick() - steer_last_command_ms) > STEER_MOTOR_CMD_TIMEOUT_MS))
        {
            SteerMotorTask_Stop();
            enabled = 0U;
        }

        /* 处理零位请求 */
        for (i = 0U; i < STEER_MOTOR_NUM; i++)
            if ((zero_mask & (uint8_t)(1U << i)) != 0U)
                SteerMotor_SetZero(i);

        /* 执行命令(命令持续有效, 直到 $STOP 或新命令覆盖).
           锁死的通道强制 0 速度保持, 只有未锁死的通道按命令模式控制. */
        if (!enabled)
        {
            SteerMotor_StopAll();
        }
        else
        {
            for (i = 0U; i < STEER_MOTOR_NUM; i++)
            {
                if ((STEER_LOCKED_CHANNEL_MASK & (1U << i)) != 0U)
                    SteerMotor_SetTargetSpeed(i, 0.0f);
                else if (speed_mode)
                    SteerMotor_SetTargetSpeed(i, speed_command[i]);
                else
                    SteerMotor_SetTargetAngle(i, angle_command[i]);
            }
        }

        SteerMotor_UpdateControl((float)STEER_MOTOR_TASK_PERIOD_MS * 0.001f);
        if ((speed_send_div_cnt++ % STEER_SPEED_SEND_DIV) == 0U)
            SteerMotor_SendSpeeds();

        /* 每 100ms 通过 USB 虚拟串口输出. JustFloat模式发二进制帧给VOFA+画曲线, 否则发$STD文本遥测 */
        {
            static uint32_t dbg_last_ms = 0U;
            if ((HAL_GetTick() - dbg_last_ms) >= 100U)
            {
                dbg_last_ms = HAL_GetTick();
#if STEER_VOFA_JUSTFLOAT
                /* float×4 + JustFloat帧尾 一次发完, 避免分两次被BUSY丢掉帧尾 */
                static uint8_t jf_buf[20];
                float *f = (float *)jf_buf;
                f[0] = (float)SteerMotor_GetMotor(0)->angle_deg;         /* CH1: 通道0角度 */
                f[1] = (float)SteerMotor_GetMotor(1)->angle_deg;         /* CH2: 通道1角度 */
                f[2] = (float)SteerMotor_GetMotor(0)->target_angle_deg;  /* CH3: 目标角度 */
                f[3] = (float)SteerMotor_GetMotor(0)->given_speed_norm;  /* CH4: 输出速度 */
                jf_buf[16] = 0x00; jf_buf[17] = 0x00;
                jf_buf[18] = 0x80; jf_buf[19] = 0x7F;
                (void)CDC_Transmit_FS(jf_buf, sizeof(jf_buf));
#else
                UsbDebug_Printf("$STD,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%lu,%u,%lu\r\n",
                    (unsigned)SteerMotor_GetMotor(0)->feedback_ready,   /* 2:  fbA */
                    (unsigned)SteerMotor_GetMotor(1)->feedback_ready,   /* 3:  fbB */
                    (unsigned)SteerMotor_GetMotor(2)->feedback_ready,   /* 4:  fbC */
                    (unsigned)SteerMotor_GetMotor(3)->feedback_ready,   /* 5:  fbD */
                    (double)SteerMotor_GetMotor(0)->angle_deg,          /* 6:  angA 通道0角度 */
                    (double)SteerMotor_GetMotor(1)->angle_deg,          /* 7:  angB 通道1角度 */
                    (double)SteerMotor_GetMotor(2)->angle_deg,          /* 8:  angC 通道2角度 */
                    (double)SteerMotor_GetMotor(3)->angle_deg,          /* 9:  angD 通道3角度 */
                    (double)SteerMotor_GetMotor(0)->target_angle_deg,   /* 10: targetA */
                    (double)SteerMotor_GetMotor(0)->given_speed_norm,   /* 11: spdA */
                    (unsigned long)steer_isr_count,                     /* 12: isr */
                    (unsigned)SteerMotor_IsOnline(0),                   /* 13: online */
                    (unsigned long)steer_tx_count);                     /* 14: tx */
#endif
            }
        }

        /* osDelayUntil 因 INCLUDE_vTaskDelayUntil=0 是空操作, 导致循环跑飞灌爆板子;
           改用 osDelay(INCLUDE_vTaskDelay=1) 保证 20ms 周期 */
        osDelay(STEER_MOTOR_TASK_PERIOD_MS);
    }
}
