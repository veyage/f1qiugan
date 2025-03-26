#include "AS5600.h"
#include "math.h"
#include "filter.h"
#define DATA_SIZE 2
 
#define PI         3.14159265359f
float angle_prev=0; 
int full_rotations=0; // full rotation tracking;
float angle_d;				//GetAngle_Without_Track()的返回值
float angle_cd;				//GetAngle()的返回值
#define AS5600_ADDRESS        0x6C 
 #define Angle_Hight_Register_Addr    0x0C
 
//发送单字节时序
void AS5600_Write_Reg(uint16_t reg, uint8_t *value)
{
	HAL_I2C_Mem_Write(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 50);
}
 
 
//发送多字节时序
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len)
{
	HAL_I2C_Mem_Write(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, len, 50);
}
 
 
//IIC读多字节
void AS5600_Read_Reg(uint16_t reg, uint8_t* buf, uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}
 
 
 
 
//得到弧度制的角度，范围在0-6.28
float GetAngle_Without_Track(void)
{   
	int16_t in_angle;
	uint8_t temp[DATA_SIZE]={0};
	AS5600_Read_Reg( Angle_Hight_Register_Addr, temp, DATA_SIZE);
	in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	
	angle_d = (float)in_angle * (2.0f*PI) / 4096;
//angle_d为弧度制，范围在0-6.28	
	return angle_d;
}
 
 
 
 
//得到弧度制的带圈数角度
float GetAngle(void)
{   GetAngle_Without_Track(); 
    float val = angle_d;
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > (0.8f*2.0f*PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
	
		angle_cd = full_rotations * (2.0f*PI) + angle_prev;
	return angle_cd;
//    return (float)full_rotations * 6.28318530718f + angle_prev;
}
 
int Last_Vel_ts = 0;
float Vel_Last_Angle = 0.0;
 float dt1; 
float GetVelocity(void)
{
    float dt1 = 0.0;
    int Vel_ts = SysTick->VAL;
    if (Vel_ts >Last_Vel_ts) 
		dt1 = (Vel_ts - Last_Vel_ts) / (reload_value/1000) * 1e-3f;
    else 
		dt1 = (reload_value - Last_Vel_ts + Vel_ts) /(reload_value/1000) * 1e-3f;
    float Vel_Angle = GetAngle();

    float dv = Vel_Angle - Vel_Last_Angle;
//   dv=Lowpassfilter(dv);
//		dt1=Lowpassfilter_t(dt1);
    float velocity = (Vel_Angle - Vel_Last_Angle) / dt1;

    Last_Vel_ts = Vel_ts;
    Vel_Last_Angle = Vel_Angle;
//    printf("%f,%f\n",dt1,velocity);
    return velocity;
}
