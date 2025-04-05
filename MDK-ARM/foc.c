#include <math.h>
#include <stdlib.h>
#include "foc.h"
#include "AS5600.h"
#include "arm_math.h" // 添加arm_math库头文件
#include "pid.h"
#include "filter.h"
#include "current.h"
#define _3PI_2     4.71238898f
#define _1_SQRT3 	 0.57735026919f
#define _2_SQRT3   1.15470053838f 


// 初始化低通滤波器
LOWPASS M0_CUR_Filter = {0.01, 0.0f, 0};
LOWPASS M0_VEL_Filter = {0.1, 0.0f, 0};
LOWPASS M1_CUR_Filter = {0.01, 0.0f, 0};
LOWPASS M1_VEL_Filter = {0.1, 0.0f, 0};
struct Motor_ Motor0;
struct Motor_ Motor1;
struct AS5600_Sensor Angle_Sensor0 = {0};
struct AS5600_Sensor Angle_Sensor1 = {1};

struct Current_Sensor Current_Sensor0 = {0};
struct Current_Sensor Current_Sensor1 = {1};
\
float voltage_limit = 12;
float voltage_power_supply = 0;
float zero_electric_Angle=0.0;
int pp = 11,Dir =1;
int M0_DIR=1;
float dutydata;
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

float electricAngle(struct Motor_ *Motor)
{
  return normalizeAngle((GetAngle_Without_Track(&Angle_Sensor0) * pp * Dir) - zero_electric_Angle);
}

void SetPwm(struct Motor_ *Motor, float Ua, float Ub, float Uc)
{
  Motor->Ua = constrain(Ua, 0.0f, voltage_limit);
  Motor->Ub = constrain(Ub, 0.0f, voltage_limit);
  Motor->Uc = constrain(Uc, 0.0f, voltage_limit);

  Motor->dc_a = constrain(Motor->Ua / voltage_power_supply, 0.0f, 1.0f);
  Motor->dc_b = constrain(Motor->Ub / voltage_power_supply, 0.0f, 1.0f);
  Motor->dc_c = constrain(Motor->Uc / voltage_power_supply, 0.0f, 1.0f);

  PWM_Channel1(Motor->dc_a * dutydata);  // 频率15k
  PWM_Channel2(Motor->dc_b * dutydata);
  PWM_Channel3(Motor->dc_c * dutydata);
}

//FOC核心算法，克拉克逆变换/帕克逆变换
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float Ud, float angle_el)
{
  Motor->Ualpha = -Uq * arm_sin_f32(angle_el); // 使用arm_sin_f32
  Motor->Ubeta = Uq * arm_cos_f32(angle_el);   // 使用arm_cos_f32

  Motor->Ua = Motor->Ualpha + voltage_power_supply / 2;
  Motor->Ub = (_2_SQRT3 * Motor->Ubeta - Motor->Ualpha) / 2 + voltage_power_supply / 2;
  Motor->Uc = -(Motor->Ualpha + _2_SQRT3 * Motor->Ubeta) / 2 + voltage_power_supply / 2;

  SetPwm(Motor, Motor->Ua, Motor->Ub, Motor->Uc);
}

void Check_Sensor(void)
{
  SetPhaseVoltage(&Motor0, 3, 0, _3PI_2);
  HAL_Delay(3000);
  zero_electric_Angle = electricAngle(&Motor0);
  SetPhaseVoltage(&Motor0, 0, 0, _3PI_2);
  HAL_Delay(500);
}

void FOC_Init(float power)
{
  voltage_power_supply = power;
  HAL_TIM_PWM_Start(&FOCTIM1, FOCTIMCHN1); 
  HAL_TIM_PWM_Start(&FOCTIM2, FOCTIMCHN2); 
  HAL_TIM_PWM_Start(&FOCTIM3, FOCTIMCHN3); 
  CurrSense_Init(&Current_Sensor0);
	dutydata =__HAL_TIM_GET_AUTORELOAD(&htim2);
//  AS5600_Init();
  Check_Sensor();
}

double shaft_angle;
double openloop_timestamp;
float velocityopenloop(float target)
{
  float Uq = 0.0;
  float Ts = 0.0;
  target = target / 1000.0;
  uint32_t now_ts  =  SysTick->VAL;

	if(now_ts> openloop_timestamp)
	  Ts = (now_ts-openloop_timestamp )/72.0f*1e-6f;
	else 
		Ts = (reload_value - now_ts+openloop_timestamp)/72.0f*1e-6f;



  shaft_angle = normalizeAngle(shaft_angle + pp*target*Ts);

  Uq = voltage_limit;

  SetPhaseVoltage(&Motor0, Uq, 0, shaft_angle);

  openloop_timestamp = now_ts;

  return Uq;
}
float cal_Iq_Id(struct Current_Sensor *Sensor, float angle_el)
{
	
  float I_alpha = Sensor->I_a;
  float I_beta = _1_SQRT3 * Sensor->I_a + _2_SQRT3 * Sensor->I_b;

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
    float Vel1 = GetVelocity(&Angle_Sensor0);
    float Vel = Lowpassfilter(&M0_CUR_Filter,Vel1);
    Target = Target / 1000.0;

    float Uq = PID_Controller(velocity_pid, Dir * (Target - Vel));
    SetPhaseVoltage(&Motor0, Uq, 0, electricAngle(&Motor0));
}

void Set_Angle(float Target,PID_Controller_t *angle_pid) {
    float angle = GetAngle(&Angle_Sensor0);
    float Uq = PID_Controller(angle_pid, (Target - Dir * angle) * 180 / PI);
    SetPhaseVoltage(&Motor0, Uq, 0, electricAngle(&Motor0));
}
float Cur_q;
float Iq;
void Set_CurTorque(float Target,PID_Controller_t *current_pid)
{
	  GetPhaseCurrent(&Current_Sensor0);
	 Cur_q = Lowpassfilter(&M0_CUR_Filter,cal_Iq_Id(&Current_Sensor0, electricAngle(&Motor0)));
//   Cur_q = cal_Iq_Id(&Current_Sensor0, electricAngle(&Motor0));
	 Iq = PID_Controller1(current_pid, Dir *(Target - Cur_q ));
	SetPhaseVoltage(&Motor0, Iq, 0, electricAngle(&Motor0));

}
void M0_Set_Velocity_Angle(float Target,PID_Controller_t *angle_pid,PID_Controller_t *velocity_pid)
{ float angle = GetAngle(&Angle_Sensor0);
	Set_Velocity(PID_Controller(angle_pid, (Target - Dir * angle) * 180 / PI),velocity_pid);
}
////////////////////////////
void M0_Set_CurTorque(float Target,PID_Controller_t *current_pid)
{
	GetPhaseCurrent(&Current_Sensor0);
	SetPhaseVoltage(&Motor0, PID_Controller(current_pid, (Target - Lowpassfilter(&M0_CUR_Filter, cal_Iq_Id(&Current_Sensor0, electricAngle(&Motor0))))),0, electricAngle(&Motor0));

}
void M0_Set_Velocity_cur(float Target,PID_Controller_t *velocity_pid,PID_Controller_t *current_pid)
{
	Angle_Sensor0.velocity = Lowpassfilter(&M0_VEL_Filter, GetVelocity(&Angle_Sensor0));
	M0_Set_CurTorque(PID_Controller(velocity_pid, M0_DIR*(constrain(Target, -voltage_limit, voltage_limit) - Angle_Sensor0.velocity)),current_pid);


}
void M0_Set_Velocity_Angle_cur(float Target,PID_Controller_t *angle_pid,PID_Controller_t *velocity_pid,PID_Controller_t *current_pid)
{
	M0_Set_Velocity_cur(PID_Controller(angle_pid, (Target - GetAngle(&Angle_Sensor0))),velocity_pid,current_pid);
}
// 平滑Uq变化，自动调整maxDelta

void M0_Set_Position_Velocity(float Target_Position, float Max_Velocity, PID_Controller_t *angle_pid, PID_Controller_t *velocity_pid) {
    // 获取当前角度
    float current_angle = GetAngle(&Angle_Sensor0);

    // 计算位置误差
    float position_error = Target_Position - Dir * current_angle;

    // 通过位置环PID计算目标速度
    float target_velocity = PID_Controller(angle_pid, position_error);

    // 限制目标速度在最大速度范围内
    target_velocity = constrain(target_velocity, -Max_Velocity, Max_Velocity);

    // 调用速度环控制目标速度
    Set_Velocity(target_velocity, velocity_pid);
	  //Set_Velocity(Target_Position, velocity_pid);
}

