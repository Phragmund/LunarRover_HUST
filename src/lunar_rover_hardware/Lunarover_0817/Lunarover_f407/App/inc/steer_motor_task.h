#ifndef __STEER_MOTOR_TASK_H__
#define __STEER_MOTOR_TASK_H__

#include "steer_motor.h"

void SteerMotorTask_Entry(void const *argument);
void SteerMotorTask_SetAngle(uint8_t index, float angle_deg);
void SteerMotorTask_SetAllAngles(const float angle_deg[STEER_MOTOR_NUM]);
void SteerMotorTask_SetSpeed(uint8_t index, float speed_norm);
void SteerMotorTask_SetAllSpeed(const float speed_norm[STEER_MOTOR_NUM]);
void SteerMotorTask_SetZero(uint8_t index);
void SteerMotorTask_SetAllZero(void);
void SteerMotorTask_Stop(void);
void SteerMotorTask_Enable(uint8_t enable);
void SteerMotorTask_CdcRxCallback(uint8_t *data, uint32_t len);

#endif
