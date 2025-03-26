#ifndef FILTER_H
#define FILTER_H

#include "arm_math.h"
extern float reload_value;
// 卡尔曼滤波器结构体
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;

// 低通滤波器函数
float Lowpassfilter(float x);
float Lowpassfilter_sim(float x);
float Lowpassfilter_v(float x);
float Lowpassfilter_t(float x);
// 卡尔曼滤波器函数
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);

// 接口函数
float  ApplyFilters(float input);

#endif // FILTER_H