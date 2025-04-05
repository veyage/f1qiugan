#ifndef CURRENT_H
#define CURRENT_H

#include "stm32f1xx_hal.h"
#include "adc.h"
#define limitvel 0.5
#define limitangle 0.4
// 电流传感器结构体
struct Current_Sensor {
    int Sen_Num;          // 传感器编号
    float I_a;            // 相电流 A
    float I_b;            // 相电流 B
    float offset_ia;      // 零飘偏移 A
    float offset_ib;      // 零飘偏移 B
    float current_a;      // 当前电流 A
    float current_b;      // 当前电流 B
    float current_c;      // 当前电流 C
};

// 外部变量声明
extern uint16_t Samp_volts[4];

// 函数声明
void Set_Cur_Sensor(int Mot_num);
void DriftOffsets(struct Current_Sensor *Sensor);
void CurrSense_Init(struct Current_Sensor *Sensor);
void GetPhaseCurrent(struct Current_Sensor *Sensor);

#endif // CURRENT_H


