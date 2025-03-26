#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#include "main.h"
#include "tim.h"
extern float reload_value;

typedef struct {
    float Kp;                  // 比例系数
    float Ki;                  // 积分系数
    float Kd;                  // 微分系数
    float Last_Error;          // 上一次误差
    float Last_Integration;    // 上一次积分值
    float Last_Output;         // 上一次输出值
    unsigned long Timestamp_Last; // 上一次时间戳
} PID_Controller_t;
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);
float PID_Controller(PID_Controller_t *pid, float Error);
float  _constrain(float amt, float low, float high);
float PID_Controller1(PID_Controller_t *pid, float Error);
#endif


