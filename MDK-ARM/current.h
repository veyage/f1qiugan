#ifndef CURRENT_H
#define CURRENT_H
#include "main.h"
#include "adc.h"
struct CurrentDetect{
	float I_a;
	float I_b;
	float U_a;
	float U_b;
};
//extern struct CurrentDetect Current;

void DriftOffsets(void);
void CurrSense_Init(void);
void GetPhaseCurrent(struct CurrentDetect *current);
#endif


