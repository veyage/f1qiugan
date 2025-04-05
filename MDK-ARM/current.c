#include "current.h"

#define _ADC_CONV    0.00080586f

float _shunt_resistor = 0;
float amp_gain = 0;
float vlots_to_amps = 0;
float gain_a, gain_b, gain_c;
float offset_ia = 0.0, offset_ib = 0.0;
float current_a, current_b, current_c;
uint16_t Samp_volts[4];	
int A, B;

// ¡„∆ÆºÏ≤‚
void DriftOffsets(struct Current_Sensor *Sensor) {
    uint16_t detect_rounds = 1000;
    for (int i = 0; i < detect_rounds; i++) {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Samp_volts, 2);
        Sensor->offset_ia += Samp_volts[A] * _ADC_CONV;
        Sensor->offset_ib += Samp_volts[B] * _ADC_CONV;
    }

    Sensor->offset_ia = Sensor->offset_ia / detect_rounds;
    Sensor->offset_ib = Sensor->offset_ib / detect_rounds;
}

void Set_Cur_Sensor(int Mot_num) {
    if (Mot_num == 0) {
        A = 0;
        B = 1;
    } else if (Mot_num == 1) {
        A = 2;
        B = 3;
    }
}

void CurrSense_Init(struct Current_Sensor *Sensor)
{
	DriftOffsets(Sensor);
	HAL_ADCEx_Calibration_Start(&hadc1);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Samp_volts, 2);
    _shunt_resistor = 0.01;
    amp_gain = 50;

    vlots_to_amps = 1.0f / _shunt_resistor / amp_gain;

    gain_a = vlots_to_amps * -1;
    gain_b = vlots_to_amps * -1;
    gain_c = vlots_to_amps;
}

void GetPhaseCurrent(struct Current_Sensor *current)
{ HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Samp_volts, 2);
    float tran_vol_a = (float)Samp_volts[A]*_ADC_CONV;
    float tran_vol_b = (float)Samp_volts[B]*_ADC_CONV;

    current->I_a = (tran_vol_a - current->offset_ia) * gain_a;
    current->I_b = (tran_vol_b - current->offset_ib) * gain_b;
}
