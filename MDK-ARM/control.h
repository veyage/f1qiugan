#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f1xx_hal.h"
#include "adc.h"
#include "foc.h"
#include "pid.h"
#include "AS5600.h"
#include "filter.h"
#include "current.h"
#include "arm_math.h"
void control(float *angleneed, float *velneed);
void predict_vel();
void get_d();
void get_vel();

#endif // CONTROL_H
