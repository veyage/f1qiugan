#ifndef FILTER_H
#define FILTER_H

#include "arm_math.h"

// 外部变量声明
extern float reload_value;

// 卡尔曼滤波器结构体
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;

// 低通滤波器结构体
typedef struct {
    float Tf;               // 时间常数
    float Last_y;           // 上一次的输出值
    uint32_t Last_Timesamp; // 上一次的时间戳
} LOWPASS;

// 低通滤波器函数
float Lowpassfilter(LOWPASS *lowpass, float x);


// 卡尔曼滤波器函数
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);


#endif // FILTER_H