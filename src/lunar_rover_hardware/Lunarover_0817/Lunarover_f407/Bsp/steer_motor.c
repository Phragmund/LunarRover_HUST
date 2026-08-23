#include "steer_motor.h"
#include "debug_uart.h"
#include <stdio.h>
#include <string.h>

#define STEER_MOTOR_RX_BUF_SIZE 128U

static SteerMotor_t steer_motor[STEER_MOTOR_NUM];
static UART_HandleTypeDef *steer_huart = 0;
static char steer_rx_buf[STEER_MOTOR_RX_BUF_SIZE];
static uint16_t steer_rx_len = 0U;

/* 过零(16位回绕)处理用的上一帧原始值 */
static int32_t steer_prev_raw[STEER_MOTOR_NUM];
static uint8_t steer_prev_valid[STEER_MOTOR_NUM];

/* ===== USART1 中断接收环形缓冲(根治轮询丢包) ===== */
#define STEER_RX_RING_SIZE 256U

static volatile uint8_t steer_rx_ring[STEER_RX_RING_SIZE];
static volatile uint16_t steer_rx_head = 0U;   /* 中断写指针 */
static volatile uint16_t steer_rx_tail = 0U;   /* 任务读指针 */

volatile uint32_t steer_isr_count = 0U;   /* 诊断: USART1 中断触发计数(每收到一字节+1) */
volatile uint32_t steer_tx_count = 0U;    /* 诊断: USART1 发送字节计数 */

/* HAL 逐字节接收缓冲: 每收 1 字节触发一次 HAL_UART_RxCpltCallback */
static volatile uint8_t steer_rx_byte;

/* USART1 中断由 CubeMX 生成: stm32f4xx_it.c 里的 USART1_IRQHandler 调用
   HAL_UART_IRQHandler(&huart1), 再由下面的 HAL 回调逐字节入环形缓冲,
   任务侧 SteerMotor_RxPoll 负责消费. 回调运行在中断上下文(优先级5,
   等于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY), 只做入环,
   不调用任何 FreeRTOS FromISR API. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart != steer_huart)
        return;

    steer_isr_count++;

    next = (uint16_t)((steer_rx_head + 1U) % STEER_RX_RING_SIZE);
    if (next != steer_rx_tail)
    {
        steer_rx_ring[steer_rx_head] = steer_rx_byte;
        steer_rx_head = next;
    }
    /* 缓冲满则丢弃该字节(head 不动), 避免覆盖任务尚未消费的数据 */

    /* 重新启动逐字节接收 */
    (void)HAL_UART_Receive_IT(huart, (uint8_t *)&steer_rx_byte, 1U);
}

/* USART1 出错(如 ORE 过载)时恢复接收, 防止中断链路卡死.
   进入此回调时 HAL 已清除错误标志并把 RxState 复位为 READY. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != steer_huart)
        return;

    (void)HAL_UART_Receive_IT(huart, (uint8_t *)&steer_rx_byte, 1U);
}

static uint8_t steer_rx_ring_pop(uint8_t *c)
{
    if (steer_rx_head == steer_rx_tail)
        return 0U;

    *c = steer_rx_ring[steer_rx_tail];
    steer_rx_tail = (uint16_t)((steer_rx_tail + 1U) % STEER_RX_RING_SIZE);
    return 1U;
}

static float steer_limitf(float x, float max, float min)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static void steer_send_string(const char *s)
{
    uint16_t len;

    if (steer_huart == 0 || s == 0)
        return;

    len = (uint16_t)strlen(s);
    steer_tx_count += len;
    (void)HAL_UART_Transmit(steer_huart, (uint8_t *)s, len, 20U);
}

static void steer_parse_encoder_total(const char *frame)
{
    int32_t value[STEER_MOTOR_DRIVER_CH_NUM];
    uint8_t i;

    if (sscanf(frame, "$MOTOR_4CH_Encoder_Total:%ld,%ld,%ld,%ld!",
               &value[0], &value[1], &value[2], &value[3]) != 4)
    {
        return;
    }

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        uint8_t ch = steer_motor[i].driver_channel;
        int32_t raw;
        int32_t delta;

        if (ch >= STEER_MOTOR_DRIVER_CH_NUM)
            continue;

        raw = value[ch];

        /* 过零(16位回绕)处理: 跳变超过半量程说明计数器回绕, 补偿一个满量程 */
        if (!steer_prev_valid[i])
        {
            steer_motor[i].encoder_total_unwrapped = raw;
            steer_prev_valid[i] = 1U;
        }
        else
        {
            delta = raw - steer_prev_raw[i];
            if (delta > 32768)
                delta -= 65536;
            else if (delta < -32768)
                delta += 65536;

            /* 异常滤波: 单次增量超物理范围 → 判定霍尔突变, 丢弃该样本.
               不累计编码器、不更新角度、prev_raw 保持上次正常值(避免连续丢帧),
               但 last_update/feedback_ready 照常更新(板子仍在线). */
            if (delta > STEER_MOTOR_ENC_MAX_DELTA || delta < -STEER_MOTOR_ENC_MAX_DELTA)
            {
                steer_motor[i].last_update_ms = HAL_GetTick();
                steer_motor[i].feedback_ready = 1U;
                continue;
            }

            steer_motor[i].encoder_total_unwrapped += delta;
        }
        steer_prev_raw[i] = raw;

        steer_motor[i].encoder_total_raw = raw;
        steer_motor[i].angle_deg =
            ((float)(steer_motor[i].encoder_total_unwrapped - steer_motor[i].zero_encoder_raw) /
             (STEER_MOTOR_ENCODER_PER_DEG * STEER_MOTOR_STEERING_RATIO)) * (float)steer_motor[i].direction;
        steer_motor[i].last_update_ms = HAL_GetTick();
        steer_motor[i].feedback_ready = 1U;
    }
}

static void steer_parse_encoder_20ms(const char *frame)
{
    int value[STEER_MOTOR_DRIVER_CH_NUM];
    uint8_t i;

    if (sscanf(frame, "$MOTOR_4CH_Encoder_20ms:%d,%d,%d,%d!",
               &value[0], &value[1], &value[2], &value[3]) != 4)
    {
        return;
    }

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        uint8_t ch = steer_motor[i].driver_channel;
        if (ch >= STEER_MOTOR_DRIVER_CH_NUM)
            continue;

        steer_motor[i].encoder_20ms_raw = (int16_t)value[ch];
        steer_motor[i].last_update_ms = HAL_GetTick();
        steer_motor[i].feedback_ready = 1U;
    }
}

static void steer_parse_byte(uint8_t byte)
{
    if (byte == '$')
    {
        steer_rx_len = 0U;
    }

    if (steer_rx_len < (STEER_MOTOR_RX_BUF_SIZE - 1U))
    {
        steer_rx_buf[steer_rx_len++] = (char)byte;
        steer_rx_buf[steer_rx_len] = '\0';
    }
    else
    {
        steer_rx_len = 0U;
        return;
    }

    if (byte == '!')
    {
        steer_parse_encoder_total(steer_rx_buf);
        steer_parse_encoder_20ms(steer_rx_buf);
        steer_rx_len = 0U;
    }
}

void SteerMotor_Init(UART_HandleTypeDef *huart)
{
    uint8_t i;

    memset(steer_motor, 0, sizeof(steer_motor));
    memset(steer_prev_raw, 0, sizeof(steer_prev_raw));
    memset(steer_prev_valid, 0, sizeof(steer_prev_valid));
    steer_huart = huart;
    steer_rx_len = 0U;
    steer_rx_head = 0U;
    steer_rx_tail = 0U;

    /* USART1 中断优先级与 NVIC 使能已由 CubeMX 在 HAL_UART_MspInit 中配置(优先级5).
       这里启动 HAL 逐字节中断接收: 每收 1 字节触发 HAL_UART_RxCpltCallback 入环. */
    (void)HAL_UART_Receive_IT(huart, (uint8_t *)&steer_rx_byte, 1U);

    /* 调试串口 USART2(PA2 TX) 用于打印原始收发帧 */
    DebugUart_Init();

    /* 每通道独立方向配置:
       direction = 角度读数符号(顺时针拨轮, 读正→+1, 读负→-1)
       motor_dir = 电机转向符号(发$ANG轮子转对方向→+1, 反→-1) */
    static const int8_t dir_table[STEER_MOTOR_NUM]  = { -1, 1, -1, 1 };  /* 通道0/1读数反取-1 */
    static const int8_t mdir_table[STEER_MOTOR_NUM] = { -1, 1, -1, 1 };   /* 通道1电机转向反取-1 */

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        steer_motor[i].driver_channel = i;
        steer_motor[i].direction = dir_table[i];
        steer_motor[i].motor_dir = mdir_table[i];
        steer_motor[i].kp = 0.008;   /* 误差30°→输出0.75, 会被限幅截到上限; 太小则轮子爬行、速度变化极慢 */
        steer_motor[i].kd = 0.00016f;    /* 微分项关掉(原始配置) */
        steer_motor[i].mode = STEER_MOTOR_MODE_RELAX;
    }

    /* 初始化四路板: 电机类型 0=520电机 1=TT马达 2=310电机.
       MG513X 舵电机必须用类型 2, 否则闭环速度控制失效(表现为电机猛转).
       与 Arduino 例程一致: 每条命令间延时, 最后发一次 0 速度. */
    steer_send_string("$MOTOR_4CH_SET:2!");
    HAL_Delay(500);
    steer_send_string("$MOTOR_4CH_SET_ENCPDER_POLARITY:0!");
    HAL_Delay(100);
    steer_send_string("$Car:0,0,0,0!");
    HAL_Delay(100);
	steer_send_string("$MOTOR_4CH_SET:2!");
	HAL_Delay(500);
	}//我也不知道为什么这样可以，但这么做就对了

/* 把一条原始帧原样发给四路板(USART1), 用于串口透传下发配置/读取指令 */
void SteerMotor_SendRaw(const char *frame)
{
    steer_send_string(frame);
}

void SteerMotor_SetTargetAngle(uint8_t index, float angle_deg)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    angle_deg = steer_limitf(angle_deg, STEER_MOTOR_MAX_ANGLE_DEG, -STEER_MOTOR_MAX_ANGLE_DEG);
    steer_motor[index].target_angle_deg = angle_deg;
    steer_motor[index].mode = STEER_MOTOR_MODE_POSITION;
}

void SteerMotor_SetTargetSpeed(uint8_t index, float speed_norm)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    steer_motor[index].target_speed_norm = steer_limitf(speed_norm, 1.0f, -1.0f);
    steer_motor[index].mode = STEER_MOTOR_MODE_SPEED;
}

void SteerMotor_SetTargetPwm(uint8_t index, float pwm_norm)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    steer_motor[index].target_pwm_norm = steer_limitf(pwm_norm, 1.0f, -1.0f);
    steer_motor[index].mode = STEER_MOTOR_MODE_PWM;
}

void SteerMotor_SetZero(uint8_t index)
{
    if (index >= STEER_MOTOR_NUM)
        return;

    steer_motor[index].zero_encoder_raw = steer_motor[index].encoder_total_unwrapped;
    steer_motor[index].angle_deg = 0.0f;
    steer_motor[index].target_angle_deg = 0.0f;
    steer_motor[index].last_error_deg = 0.0f;
}

void SteerMotor_SetAllZero(void)
{
    uint8_t i;

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
        SteerMotor_SetZero(i);
}

void SteerMotor_StopAll(void)
{
    uint8_t i;

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        steer_motor[i].target_speed_norm = 0.0f;
        steer_motor[i].target_pwm_norm = 0.0f;
        steer_motor[i].given_speed_norm = 0.0f;
        steer_motor[i].last_error_deg = 0.0f;
        steer_motor[i].mode = STEER_MOTOR_MODE_SPEED;
    }
}

void SteerMotor_RelaxAll(void)
{
    uint8_t i;

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        steer_motor[i].target_speed_norm = 0.0f;
        steer_motor[i].target_pwm_norm = 0.0f;
        steer_motor[i].given_speed_norm = 0.0f;
        steer_motor[i].last_error_deg = 0.0f;
        steer_motor[i].mode = STEER_MOTOR_MODE_RELAX;
    }
}

void SteerMotor_RequestEncoderTotal(void)
{
    steer_send_string("$MOTOR_4CH_READ:encoder_total!");
}

void SteerMotor_RequestEncoder20ms(void)
{
    steer_send_string("$MOTOR_4CH_READ:encoder_20ms!");
}

void SteerMotor_RxPoll(uint32_t timeout_ms)
{
    uint8_t byte;

    (void)timeout_ms;
    if (steer_huart == 0)
        return;

    /* 非阻塞: 把环形缓冲里已到的字节消费完就返回, 不空转等待.
       原来 do-while 会把 timeout 整个空转掉, 任务实际 ~10Hz,
       反馈更新很慢; 改后任务回到 20ms 周期, 反馈 ~50Hz. */
    while (steer_rx_ring_pop(&byte))
        steer_parse_byte(byte);
}

void SteerMotor_UpdateControl(float dt_s)
{
    uint8_t i;

    for (i = 0U; i < STEER_MOTOR_NUM; i++)
    {
        SteerMotor_t *motor = &steer_motor[i];

        if (motor->mode == STEER_MOTOR_MODE_POSITION)
        {
            float error = motor->target_angle_deg - motor->angle_deg;
            float derivative = (dt_s > 0.000001f) ? ((error - motor->last_error_deg) / dt_s) : 0.0f;

            if (!SteerMotor_IsOnline(i))
            {
                motor->given_speed_norm = 0.0f;
            }
            else if (error < STEER_MOTOR_POS_DEADBAND_DEG && error > -STEER_MOTOR_POS_DEADBAND_DEG)
            {
                motor->given_speed_norm = 0.0f;
            }
            else
            {
                motor->given_speed_norm = motor->kp * error + motor->kd * derivative;
                motor->given_speed_norm = steer_limitf(motor->given_speed_norm,
                                                       STEER_MOTOR_POS_MAX_SPEED, -STEER_MOTOR_POS_MAX_SPEED);
            }

            motor->last_error_deg = error;
        }
        else if (motor->mode == STEER_MOTOR_MODE_SPEED)
        {
            motor->given_speed_norm = motor->target_speed_norm;
        }
        else
        {
            motor->given_speed_norm = 0.0f;
        }
    }
}

void SteerMotor_SendSpeeds(void)
{
    char tx[96];

    (void)snprintf(tx, sizeof(tx), "$Car:%.3f,%.3f,%.3f,%.3f!",
                   steer_motor[0].given_speed_norm * (float)steer_motor[0].motor_dir,
                   steer_motor[1].given_speed_norm * (float)steer_motor[1].motor_dir,
                   steer_motor[2].given_speed_norm * (float)steer_motor[2].motor_dir,
                   steer_motor[3].given_speed_norm * (float)steer_motor[3].motor_dir);
    steer_send_string(tx);
}

void SteerMotor_SendPwms(void)
{
    char tx[96];

    (void)snprintf(tx, sizeof(tx), "$Car_Pwm:%.3f,%.3f,%.3f,%.3f!",
                   steer_motor[0].target_pwm_norm,
                   steer_motor[1].target_pwm_norm,
                   steer_motor[2].target_pwm_norm,
                   steer_motor[3].target_pwm_norm);
    steer_send_string(tx);
}

uint8_t SteerMotor_IsOnline(uint8_t index)
{
    if (index >= STEER_MOTOR_NUM)
        return 0U;

    if (!steer_motor[index].feedback_ready)
        return 0U;

    return ((HAL_GetTick() - steer_motor[index].last_update_ms) <= STEER_MOTOR_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
}

const SteerMotor_t *SteerMotor_GetMotor(uint8_t index)
{
    if (index >= STEER_MOTOR_NUM)
        return 0;

    return &steer_motor[index];
}
