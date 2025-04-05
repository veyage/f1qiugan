#include "AS5600.h"
#include "math.h"
#include "filter.h"

#define DATA_SIZE 2
#define PI         3.14159265359f
#define _2PI       6.28318530718f
#define AS5600_ADDRESS        0x6C
#define Angle_Hight_Register_Addr    0x0C

// 发送单字节时序
void AS5600_Write_Reg(uint16_t reg, uint8_t *value) {
    HAL_I2C_Mem_Write(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 50);
}

// 发送多字节时序
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len) {
    HAL_I2C_Mem_Write(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, len, 50);
}

// IIC 读多字节
void AS5600_Read_Reg(uint16_t reg, uint8_t *buf, uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}

// 得到弧度制的角度，范围在 0-6.28
float GetAngle_Without_Track(struct AS5600_Sensor *sensor) {
    int16_t in_angle;
    uint8_t temp[DATA_SIZE] = {0};
    AS5600_Read_Reg(Angle_Hight_Register_Addr, temp, DATA_SIZE);
    in_angle = ((int16_t)temp[0] << 8) | (temp[1]);

    sensor->Angle = (float)in_angle * (2.0f * PI) / 4096;
    // 返回弧度制角度，范围在 0-6.28
    return sensor->Angle;
}

// 得到弧度制的带圈数角度
float GetAngle(struct AS5600_Sensor *sensor) {
    float val = GetAngle_Without_Track(sensor);
    float d_angle = val - sensor->Last_Angle;

    // 计算旋转的总圈数
    if (fabs(d_angle) > (0.8f * 2.0f * PI)) {
        sensor->full_rotations += (d_angle > 0) ? -1 : 1;
    }
    sensor->Last_Angle = val;

    sensor->Angle = sensor->full_rotations * (2.0f * PI) + sensor->Last_Angle;
    return sensor->Angle;
}

// 获取速度
float GetVelocity(struct AS5600_Sensor *sensor) {
    
    int Vel_ts = SysTick->VAL;
    float dt = 0.0;
    if (Vel_ts > sensor->Last_Vel_ts) {
        dt = (Vel_ts - sensor->Last_Vel_ts) / (reload_value / 1000) * 1e-3f;
    } else {
        dt = (reload_value - sensor->Last_Vel_ts + Vel_ts) / (reload_value / 1000) * 1e-3f;
    }

    if (dt < 0.0001f) {
        dt = 0.0001f; // 防止除以零
    }
  
    float Vel_Angle = GetAngle(sensor);
    float dv = Vel_Angle - sensor->Vel_Last_Angle;

    sensor->velocity = dv / dt;
    sensor->Last_Vel_ts = Vel_ts;
    sensor->Vel_Last_Angle = Vel_Angle;

    return sensor->velocity;
}
