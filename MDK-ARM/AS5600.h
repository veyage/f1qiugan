#ifndef AS5600_H
#define AS5600_H

#include "stm32f1xx_hal.h"
#include "arm_math.h"
#include "i2c.h"

// AS5600 传感器结构体
struct AS5600_Sensor {
    int Mot_num;           // 电机编号
    float Angle;           // 当前角度
    float Last_Angle;      // 上一次角度
    int full_rotations;    // 完整旋转圈数
    float velocity;        // 当前速度
    int Last_Vel_ts;       // 上一次速度时间戳
    float Vel_Last_Angle;  // 上一次速度角度
};

// 函数声明
void AS5600_Write_Reg(uint16_t reg, uint8_t *value);
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len);
void AS5600_Read_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
float GetAngle_Without_Track(struct AS5600_Sensor *sensor);
float GetAngle(struct AS5600_Sensor *sensor);
float GetVelocity(struct AS5600_Sensor *sensor);

#endif // AS5600_H