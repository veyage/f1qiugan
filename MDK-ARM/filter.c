#include "filter.h"
#include "stm32f1xx_hal.h"

/////////////////ÂË²¨

KalmanFilter kalman_filter;
uint32_t Last_Timesamp = 0.0;
float Last_y = 0.0;
float alpha = 0.998;
float y;
float Tf=0.4;
float Lowpassfilter(float x) {
    float dt = 0.0;

    uint32_t Timesamp = SysTick->VAL;
    if (Timesamp > Last_Timesamp) 
        dt = (float)(Timesamp - Last_Timesamp) / (reload_value/1000) * 1e-3f;
    else
        dt = (float)(reload_value   - Last_Timesamp + Timesamp) / (reload_value/1000) * 1e-3f;

		
    alpha = Tf / (Tf + dt);
    y = alpha * Last_y + (1.0f - alpha) * x;

    Last_y = y;
    Last_Timesamp = Timesamp;

    return y;
}

uint32_t Last_Timesamp0 = 0.0;
float Last_y0 = 0.0;
float alpha0 = 0.998;
float y0;
float Tf0=0.4;
float Lowpassfilter_v(float x) {
    float dt = 0.0;

    uint32_t Timesamp = SysTick->VAL;
    if (Timesamp < Last_Timesamp0) 
        dt = (float)(Last_Timesamp0 - Timesamp) / (reload_value/1000) * 1e-3;
    else
        dt = (float)(reload_value   - Timesamp + Last_Timesamp0) / (reload_value/1000) * 1e-3;

		
    alpha0 = Tf0 / (Tf0 + dt);
    y0 = alpha0 * Last_y0 + (1.0f - alpha0) * x;

    Last_y0 = y0;
    Last_Timesamp0 = Timesamp;

    return y0;
}
uint32_t Last_Timesamp1 = 0.0;
float Last_y1 = 0.0;
float alpha1 = 0.998;
float y1;
float Tf1=0.4;
float Lowpassfilter_t(float x) {
    float dt = 0.0;

    uint32_t Timesamp = SysTick->VAL;
    if (Timesamp > Last_Timesamp1) 
        dt = (float)(Timesamp - Last_Timesamp1) / (reload_value/1000) * 1e-3;
    else
        dt = (float)(reload_value   - Last_Timesamp1 + Timesamp) / (reload_value/1000)  * 1e-3;

		
    alpha1 = Tf1 / (Tf1 + dt);
    y1 = alpha1 * Last_y1 + (1.0f - alpha1) * x;

    Last_y1 = y1;
    Last_Timesamp1 = Timesamp;

    return y1;
}

float Lowpassfilter_sim(float x) {
    float out = 0.9 * x + 0.1 * y;
    y = x;
    return out;
}

// ¿¨¶ûÂüÂË²¨Æ÷³õÊ¼»¯
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

// ¿¨¶ûÂüÂË²¨Æ÷¸üĞÂ
float KalmanFilter_Update(KalmanFilter *filter, float measurement) {
    // Ô¤²â¸üĞÂ
    filter->p = filter->p + filter->q;

    // ¼ÆËã¿¨¶ûÂüÔöÒæ
    filter->k = filter->p / (filter->p + filter->r);

    // ¸üĞÂ¹À¼ÆÖµ
    filter->x = filter->x + filter->k * (measurement - filter->x);

    // ¸üĞÂ¹À¼ÆÎó²îĞ­·½²î
    filter->p = (1.0f - filter->k) * filter->p;

    return filter->x;
}

// ½Ó¿Úº¯Êı

float ApplyFilters(float input) { 

    static int kalman_initialized = 2;

    // µÍÍ¨ÂË²¨Æ÷
    float lowpass_output = Lowpassfilter(input);

	// ¿¨¶ûÂüÂË²¨Æ÷
	if (!kalman_initialized) {
			KalmanFilter_Init(&kalman_filter, 0.02, 2, lowpass_output);
		kalman_initialized=1;
	}
	if (kalman_initialized==1)
	return KalmanFilter_Update(&kalman_filter, lowpass_output);
	else
	return lowpass_output;
//		// ¿¨¶ûÂüÂË²¨Æ÷
//	if (!kalman_initialized) {
//			KalmanFilter_Init(&kalman_filter, 0.1f, 0.1f, input);
//		kalman_initialized=1;
//	}
//	printf("%.2f\n",KalmanFilter_Update(&kalman_filter, input));
//	return KalmanFilter_Update(&kalman_filter, input);
//		printf("%.2f\n",lowpass_output);
//  	return lowpass_output;
}
