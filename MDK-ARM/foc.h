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

// 定义 Motor_ 和 Current_Sensor 结构体
struct Motor_ {
    int Mot_num;
    float Ua;
    float Ub;
    float Uc;
    float Ualpha;
    float Ubeta;
    float dc_a;
    float dc_b;
    float dc_c;
};



// 外部变量声明
extern float reload_value;
extern struct Motor_ Motor0, Motor1;
extern struct Current_Sensor Current_Sensor0, Current_Sensor1;

// 函数声明
void Motor_en(void);
float constrain(float amt, float low, float high);
void SetPwm(struct Motor_ *Motor, float Ua, float Ub, float Uc);
float normalizeAngle(float angle);
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float Ud, float angle_el);
void Check_Sensor(void);
void FOC_Init(float power);
float electricAngle(struct Motor_ *Motor);
float velocityopenloop(float target);
void Set_Velocity(float Target, PID_Controller_t *velocity_pid);
void Set_Angle(float Target, PID_Controller_t *angle_pid);
float cal_Iq_Id(struct Current_Sensor *Sensor, float angle_el);
void Set_CurTorque(float Target, PID_Controller_t *current_pid);
void M0_Set_Velocity_Angle(float Target, PID_Controller_t *angle_pid, PID_Controller_t *velocity_pid);
void M0_Set_Velocity_Angle_cur(float Target,PID_Controller_t *angle_pid,PID_Controller_t *velocity_pid,PID_Controller_t *current_pid);
void M0_Set_Velocity_cur(float Target,PID_Controller_t *velocity_pid,PID_Controller_t *current_pid);
void M0_Set_CurTorque(float Target,PID_Controller_t *current_pid);
void M0_Set_Position_Velocity(float Target_Position, float Max_Velocity, PID_Controller_t *angle_pid, PID_Controller_t *velocity_pid) ;
#endif


