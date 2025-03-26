#include <math.h>
#include <stdlib.h>
#include "foc.h"
#include "AS5600.h"
#include "arm_math.h" // 添加arm_math库头文件
#include "pid.h"
#include "filter.h"
#include "current.h"
#define PI         3.14159265359f
#define _3PI_2     4.71238898f
#define _1_SQRT3 	 0.57735026919f
#define _2_SQRT3   1.15470053838f 

float Ua=0,Ub=0,Uc=0,Ualpha,Ubeta=0,dc_a=0,dc_b=0,dc_c=0;
float voltage_limit = 12.6;
float voltage_power_supply = 0;
float zero_electric_Angle=0.0;
float SmoothUq(float currentUq);
struct CurrentDetect Current;
int pp = 7,Dir =1;

void PWM_Channel1(float Compare)
{
  __HAL_TIM_SET_COMPARE(&FOCTIM1, FOCTIMCHN1, Compare-1);
}

void PWM_Channel2(float Compare)
{
  __HAL_TIM_SET_COMPARE(&FOCTIM2, FOCTIMCHN2, Compare-1);
}

void PWM_Channel3(float Compare)
{
  __HAL_TIM_SET_COMPARE(&FOCTIM3, FOCTIMCHN3, Compare-1);
}

void Motor_en()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

//限制幅值
float constrain(float amt, float low, float high)
{
  return ((amt<low)?(low):((amt)>(high)?(high):(amt)));
}

//将角度归化到0-2PI
float normalizeAngle(float angle)
{
  float a = fmodf(angle, 2*PI);
  return ((a>=0) ? a : (a + 2*PI));
}

float electricAngle(void)
{
  return normalizeAngle((GetAngle_Without_Track() * pp * Dir) - zero_electric_Angle);
}

void SetPwm(float Ua, float Ub, float Uc)
{
  float U_a=0.0;
  float U_b=0.0;
  float U_c=0.0;

  U_a = constrain(Ua, 0.0f, voltage_limit);
  U_b = constrain(Ub, 0.0f, voltage_limit);
  U_c = constrain(Uc, 0.0f, voltage_limit);

  dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
  dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
  dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);

  PWM_Channel1(dc_a * 4500.0f);  // 频率15k
  PWM_Channel2(dc_b * 4500.0f);
  PWM_Channel3(dc_c * 4500.0f);
}

//FOC核心算法，克拉克逆变换/帕克逆变换
void SetPhaseVoltage(float Uq, float Ud, float angle_el)
{
//  angle_el = normalizeAngle(angle_el);

  Ualpha = -Uq * arm_sin_f32(angle_el); // 使用arm_sin_f32
  Ubeta = Uq * arm_cos_f32(angle_el);   // 使用arm_cos_f32

  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (_2_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = -(Ualpha + _2_SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

  SetPwm(Ua, Ub, Uc);
}

void Check_Sensor(void)
{
  SetPhaseVoltage(3, 0, _3PI_2);
  HAL_Delay(3000);
  zero_electric_Angle = electricAngle();
  SetPhaseVoltage(0, 0, _3PI_2);
  HAL_Delay(500);
}

void FOC_Init(float power)
{
  voltage_power_supply = power;
  HAL_TIM_PWM_Start(&FOCTIM1, FOCTIMCHN1); 
  HAL_TIM_PWM_Start(&FOCTIM2, FOCTIMCHN2); 
  HAL_TIM_PWM_Start(&FOCTIM3, FOCTIMCHN3); 
  CurrSense_Init();
//  AS5600_Init();
  Check_Sensor();
}

double shaft_angle;
double openloop_timestamp;
float velocityopenloop(float target)
{
  float Uq = 0.0;
  float Ts = 0.0;

  uint32_t now_ts  =  SysTick->VAL;

  if(now_ts < openloop_timestamp)
    Ts = (openloop_timestamp - now_ts)/(reload_value/1000)*1e-3f;
  else 
    Ts = (reload_value - now_ts + openloop_timestamp)/(reload_value/1000)*1e-3f;



  shaft_angle = normalizeAngle(shaft_angle + pp*target*Ts);

  Uq = voltage_limit;

  SetPhaseVoltage(Uq, 0, shaft_angle);

  openloop_timestamp = now_ts;

  return Uq;
}
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
	
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = arm_cos_f32(angle_el);
  float st = arm_sin_f32(angle_el);
  float I_q = I_beta * ct - I_alpha * st;
	
  return I_q;
}

extern float Kp;
extern float Ki;
extern float Kd;

#include "pid.h"



void Set_Velocity(float Target,PID_Controller_t *velocity_pid) {
    float Vel1 = GetVelocity();
    float Vel = ApplyFilters(Vel1);
    Target = Target / 1000.0;

    float Uq = PID_Controller(velocity_pid, Dir * (Target - Vel));
    SetPhaseVoltage(Uq, 0, electricAngle());
}

void Set_Angle(float Target,PID_Controller_t *angle_pid) {
    float angle = GetAngle();
    float Uq = PID_Controller(angle_pid, (Target - Dir * angle) * 180 / PI);
    SetPhaseVoltage(Uq, 0, electricAngle());
}
float Cur_q;
float Iq;
void Set_CurTorque(float Target,PID_Controller_t *current_pid)
{
	  GetPhaseCurrent(&Current);
	 Cur_q = Lowpassfilter(cal_Iq_Id(Current.I_a, Current.I_b, electricAngle()));
//   Cur_q = cal_Iq_Id(Current.I_a, Current.I_b, electricAngle());
	 Iq = PID_Controller1(current_pid, Dir *(Target - Cur_q ));
	SetPhaseVoltage(Iq, 0, electricAngle());

}
// 平滑Uq变化，自动调整maxDelta

