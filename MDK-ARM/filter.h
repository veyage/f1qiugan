#ifndef FILTER_H
#define FILTER_H

#include "arm_math.h"
extern float reload_value;
// �������˲����ṹ��
typedef struct {
    float q; // ��������Э����
    float r; // ��������Э����
    float x; // ����ֵ
    float p; // �������Э����
    float k; // ����������
} KalmanFilter;

// ��ͨ�˲�������
float Lowpassfilter(float x);
float Lowpassfilter_sim(float x);
float Lowpassfilter_v(float x);
float Lowpassfilter_t(float x);
// �������˲�������
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);

// �ӿں���
float  ApplyFilters(float input);

#endif // FILTER_H