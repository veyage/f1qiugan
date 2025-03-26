#ifndef __FOC_H
#define __FOC_H
#include "tim.h"
#define FOCTIM1 htim2
#define FOCTIM2 htim2
#define FOCTIM3 htim2
#define FOCTIMCHN1 TIM_CHANNEL_1
#define FOCTIMCHN2 TIM_CHANNEL_2
#define FOCTIMCHN3 TIM_CHANNEL_3
#define ENAPIN   GPIO_PIN_0
#define ENAGPIO     GPIOA
#include "pid.h"
extern float reload_value;
void Motor_en(void);
float constrain(float amt, float low, float high);
void SetPwm(float Ua, float Ub, float Uc);
float normalizeAngle(float angle);
void SetPhaseVoltage(float Uq, float Ud, float angle_el);
void Check_Sensor(void);
void FOC_Init(float power);
float electricAngle(void);
float GetCommand(void);
float velocityopenloop(float target);
void Set_Velocity(float Target,PID_Controller_t *velocity_pid);
void Set_Angle(float Target,PID_Controller_t *angle_pid);
float cal_Iq_Id(float current_a,float current_b,float angle_el);
void Set_CurTorque(float Target,PID_Controller_t *current_pid);
#endif


