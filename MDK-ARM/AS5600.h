#ifndef __AS5600_H
#define __AS5600_H
#include "main.h"
#include "i2c.h"
extern float reload_value;

void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t  AS5600_ReadReg(uint8_t RegAddress);
uint8_t  AS5600_ReadNowReg(void);
void AS5600_Write_Reg(uint16_t reg, uint8_t *value);
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len);
void AS5600_Read_Reg(uint16_t reg, uint8_t* buf, uint8_t len);
float GetAngle_Without_Track(void);
float GetAngle(void);
void Track(void);
float GetVelocity(void);

#endif
