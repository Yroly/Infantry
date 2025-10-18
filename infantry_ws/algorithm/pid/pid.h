#pragma once

#include "stdint.h"

class Class_PID {
public:
  void Init(float kp, float ki, float kd, float kf, float outMax,
            float integralLimit, float dt, float deadband);
  void reset(float kp, float ki, float kd, float kf);

  void calculate(float Measure, float Target);

protected:
  float Kp = 0.0f;            //!<@brief 比例系数
  float Ki = 0.0f;            //!<@brief 积分系数
  float Kd = 0.0f;            //!<@brief 微分系数
  float Kf = 0.0f;            //!<@brief 前馈系数
  float outMax = 0.0f;        //!<@brief 输出限幅
  float integralLimit = 0.0f; //!<@brief 积分限幅
  float target = 0.0f;        //!<@brief 目标值
  float measure = 0.0f;       //!<@brief 测量值
  float output = 0.0f;        //!<@brief 输出值
  float integralerror = 0.0f; //!<@brief 积分误差
  float deadband = 0.0f;      //!<@brief 死区
  float dt = 0.0f;            //!<@brief 采样周期
  float pre_target = 0.0f;    //!<@brief 上次目标值
  float pre_measure = 0.0f;   //!<@brief 上次测量值
  float pre_output = 0.0f;    //!<@brief 上次输出值
  float pre_error = 0.0f;     //!<@brief 上次误差值
};
