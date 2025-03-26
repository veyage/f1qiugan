#include "pid.h"

#define  limit            5
#define  Output_ramp      10000

// 限幅
float _constrain(float amt, float low, float high)    
{
    return ((amt < low) ? (low) : ((amt > high) ? (high) : (amt)));
}

// 定义 PID 控制器结构体


// 初始化 PID 控制器
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Last_Error = 0.0f;
    pid->Last_Integration = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Timestamp_Last = 0;
}

// PID 控制器函数
float PID_Controller(PID_Controller_t *pid, float Error) {
    float Ts = 0.0f;
    uint32_t Timestamp = SysTick->VAL;
    if (Timestamp > pid->Timestamp_Last) {
        Ts = (float)(Timestamp - pid->Timestamp_Last) / (reload_value / 1000) * 1e-3;
    } else {
        Ts = (reload_value - pid->Timestamp_Last + Timestamp) / (reload_value / 1000) * 1e-3;
    }

    float proportion = pid->Kp * Error; // P环

    float integration = pid->Last_Integration + pid->Ki * 0.5f * Ts * Error; // I环
    integration = _constrain(integration, -limit, limit);

    float differential = pid->Kd * (Error - pid->Last_Error) / Ts; // D环

    float Output = proportion + integration + differential;
    Output = _constrain(Output, -limit, limit);

    pid->Last_Error = Error;
    pid->Last_Integration = integration;
    pid->Last_Output = Output;
    pid->Timestamp_Last = Timestamp;

    return Output;
}
float PID_Controller1(PID_Controller_t *pid, float Error) {
    float Ts = 0.0f;
    uint32_t Timestamp = SysTick->VAL;
    if (Timestamp > pid->Timestamp_Last) {
        Ts = (float)(Timestamp - pid->Timestamp_Last) / (reload_value / 1000) * 1e-6;
    } else {
        Ts = (reload_value - pid->Timestamp_Last + Timestamp) / (reload_value / 1000) * 1e-6;
    }

    float proportion = pid->Kp * Error; // P环

    float integration = pid->Last_Integration + pid->Ki * 0.5f * Ts * Error; // I环
    integration = _constrain(integration, -limit, limit);

    float differential = pid->Kd * (Error - pid->Last_Error) / Ts; // D环

    float Output = proportion + integration + differential;
    Output = _constrain(Output, -limit, limit);

    pid->Last_Error = Error;
    pid->Last_Integration = integration;
    pid->Last_Output = Output;
    pid->Timestamp_Last = Timestamp;

    return Output;
}