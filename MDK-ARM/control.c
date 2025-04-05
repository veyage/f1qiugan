#include "control.h"
#include <stdint.h>

// 定义 PID 参数结构体
typedef struct {
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float prev_error; // 上一次误差
    float integral;   // 积分项
} PID_Params;
float _constrain(float amt, float low, float high)    
{
    return ((amt < low) ? (low) : ((amt > high) ? (high) : (amt)));
}
// 初始化 PID 参数
PID_Params pid_params1 = {0.5f, 0.05f, 0.0f, 0.0f, 0.0f};
PID_Params pid_params2 = {1.0f, 0.002f, 0.1f, 0.0f, 0.0f};

// 增量式 PID 控制器函数，支持传入不同的 PID 参数
float pid(float setpoint, float measured_value, PID_Params *params) {
    static float prev_output = 0.0f; // 上一次输出
    float error = setpoint - measured_value; // 当前误差
    float delta_error = error - params->prev_error; // 误差增量

    // 计算增量
    float delta_output = params->kp * delta_error +
                         params->ki * error +
                         params->kd * (delta_error - params->prev_error);

    // 更新输出
    float output = prev_output + delta_output;

    // 更新状态
    params->prev_error = error;
    prev_output = output;

    return output;
}

float pre_vel;
float pre_d;
float vel;
float d,Last_d;
float u=0.1;
float setd=0.0;
float setv=0.0;
float pidd;
float pidv;
float a=10,b=1;
float i=-0.5;
float j=0.1;
float baseangel=2.65;
float angle_from_acc;
void control(float *angleneed, float *velneed)
{
    get_d();
    get_vel();
    predict_vel(); 
    pidd = pid(setd, pre_d, &pid_params1);
	  pidd=pidd*i;
//    pidv = pid(pidd, pre_vel, &pid_params2);

    // 假设杆子的加速度 pidv 转换为角度（弧度制）
    float g = 9.8f; // 重力加速度，单位 m/s^2
    angle_from_acc = asinf(pidd / g); // 将加速度转换为角度（弧度制）
    *angleneed += angle_from_acc;
    *angleneed	=_constrain(*angleneed,2.25, 3.05);// 使用转换后的角度值更新 angleneed
 //   *velneed = _constrain((pre_vel * a + (setd - pre_d) * b) * j, -limitvel, limitvel);
	  if(*velneed <0)
			*velneed =-*velneed ;
    Last_d = d;
}
extern struct AS5600_Sensor Angle_Sensor0;
float angle;
float ag;
float dt = 0.0;
int Last_Vel_ts;
void predict_vel()
{
    angle=Angle_Sensor0.Angle;
    int Vel_ts = SysTick->VAL;

    if (Vel_ts > Last_Vel_ts) {
        dt = (Vel_ts - Last_Vel_ts) / (reload_value / 1000) * 1e-6f;
    } else {
        dt = (reload_value - Last_Vel_ts + Vel_ts) / (reload_value / 1000) * 1e-6f;
    }
  ag=baseangel-angle;
  if(vel<0&&ag>0)
  {pre_vel=(-sinf(ag)+cosf(ag)*u)*0.98*dt+vel;}
  if(vel<0&&ag<0)
  {pre_vel=(sinf(ag)+cosf(ag)*u)*0.98*dt+vel;}
  if(vel>0&&ag>0)
  {pre_vel=(-sinf(ag)-cosf(ag)*u)*0.98*dt+vel;}
  if(vel>0&&ag<0)
  {pre_vel=(sinf(ag)-cosf(ag)*u)*0.98*dt+vel;}
    pre_d=(pre_vel+vel)/2*dt+d;

}
void get_vel()
{

}
void get_d()
{

}