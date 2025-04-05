#include "filter.h"
#include "stm32f1xx_hal.h"

/////////////////滤波

extern float reload_value;


KalmanFilter kalman_filter;

// 使用 LOWPASS 结构体的低通滤波器实现
float Lowpassfilter( LOWPASS *lowpass, float x) {
    float dt = 0.0;

    uint32_t Timesamp = SysTick->VAL;
    if (Timesamp < lowpass->Last_Timesamp) {
        dt = (float)(lowpass->Last_Timesamp - Timesamp) / (reload_value / 1000) * 1e-3f;
    } else {
        dt = (float)(reload_value - Timesamp + lowpass->Last_Timesamp) / (reload_value / 1000) * 1e-3f;
    }

    if (dt <= 0.0f || dt > 0.005f) {
        lowpass->Last_y = x;
        lowpass->Last_Timesamp = Timesamp;
        return x;
    }

    float alpha = lowpass->Tf / (lowpass->Tf + dt);
    float y = alpha * lowpass->Last_y + (1.0f - alpha) * x;

    lowpass->Last_y = y;
    lowpass->Last_Timesamp = Timesamp;

    return y;
}



// 卡尔曼滤波器初始化
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

// 卡尔曼滤波器更新
float KalmanFilter_Update(KalmanFilter *filter, float measurement) {
    // 预测更新
    filter->p = filter->p + filter->q;

    // 计算卡尔曼增益
    filter->k = filter->p / (filter->p + filter->r);

    // 更新估计值
    filter->x = filter->x + filter->k * (measurement - filter->x);

    // 更新估计误差协方差
    filter->p = (1.0f - filter->k) * filter->p;

    return filter->x;
}


