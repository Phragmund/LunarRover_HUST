#ifndef __STEER_MOTOR_H__
#define __STEER_MOTOR_H__

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define STEER_MOTOR_NUM                 4U
#define STEER_MOTOR_DRIVER_CH_NUM       4U
#define STEER_MOTOR_FEEDBACK_TIMEOUT_MS 200U
#define STEER_MOTOR_CMD_TIMEOUT_MS      200U
#define STEER_MOTOR_ENCODER_PER_DEG     4.044f  /* 电机输出轴每度编码器数: 霍尔13线×4倍频=52/转子圈 × 减速比28 = 1456/输出圈 /360 */
#define STEER_MOTOR_STEERING_RATIO      2.0f    /* 转向减速比(实测修正: 原3.0导致真实转角比读数大1.5倍, 改为2.0) */
#define STEER_MOTOR_MAX_ANGLE_DEG       135.0f  /* 轮舵角度限幅(度) */
#define STEER_MOTOR_POS_DEADBAND_DEG    1.0f    /* 轮舵到位死区(度) */
#define STEER_MOTOR_POS_MAX_SPEED       0.2    /* 位置环输出速度限幅(-1~1) */
#define STEER_MOTOR_ENC_MAX_DELTA       750/* 编码器单次增量异常阈值: 超过则判定为霍尔突变, 丢弃该样本(约60ms采样) */

typedef enum
{
    STEER_MOTOR_MODE_RELAX = 0,
    STEER_MOTOR_MODE_POSITION,
    STEER_MOTOR_MODE_SPEED,
    STEER_MOTOR_MODE_PWM,
} SteerMotor_Mode_t;

typedef struct
{
    uint8_t driver_channel;
    int8_t direction;      /* 角度读数符号 ±1 */
    int8_t motor_dir;      /* 电机旋转方向符号 ±1(单独翻电机转向用) */
    int32_t zero_encoder_raw;

    volatile int32_t encoder_total_raw;
    int32_t encoder_total_unwrapped;   /* 过零(16位回绕)处理后的连续累计值 */
    volatile int16_t encoder_20ms_raw;
    volatile uint32_t last_update_ms;
    volatile uint8_t feedback_ready;

    float angle_deg;
    float target_angle_deg;
    float target_speed_norm;
    float target_pwm_norm;
    float given_speed_norm;
    float kp;
    float kd;
    float last_error_deg;
    SteerMotor_Mode_t mode;
} SteerMotor_t;

void SteerMotor_Init(UART_HandleTypeDef *huart);
void SteerMotor_SendRaw(const char *frame);
void SteerMotor_SetTargetAngle(uint8_t index, float angle_deg);
void SteerMotor_SetTargetSpeed(uint8_t index, float speed_norm);
void SteerMotor_SetTargetPwm(uint8_t index, float pwm_norm);
void SteerMotor_SetZero(uint8_t index);
void SteerMotor_SetAllZero(void);
void SteerMotor_StopAll(void);
void SteerMotor_RelaxAll(void);
void SteerMotor_RequestEncoderTotal(void);
void SteerMotor_RequestEncoder20ms(void);
void SteerMotor_RxPoll(uint32_t timeout_ms);
void SteerMotor_UpdateControl(float dt_s);
void SteerMotor_SendSpeeds(void);
void SteerMotor_SendPwms(void);
uint8_t SteerMotor_IsOnline(uint8_t index);
const SteerMotor_t *SteerMotor_GetMotor(uint8_t index);

extern volatile uint32_t steer_isr_count;   /* 诊断: USART1 中断触发计数 */
extern volatile uint32_t steer_tx_count;    /* 诊断: USART1 发送字节计数 */

#endif
