#include "pid.h"

#define LIMIT_MAX_MIN(x, max, min)                                             \
  (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#define ABS(x) ((x) > 0 ? (x) : (-(x)))

void Class_PID::Init(float kp, float ki, float kd, float kf, float outMax,
                     float integralLimit, float dt, float deadband) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  Kf = kf;
  this->outMax = outMax;
  this->integralLimit = integralLimit;
  this->dt = dt;
  this->deadband = deadband;
}

void Class_PID::reset(float kp, float ki, float kd, float kf) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  Kf = kf;

  target = 0.0f;
  measure = 0.0f;
  output = 0.0f;
  integralerror = 0.0f;
  pre_target = 0.0f;
  pre_measure = 0.0f;
  pre_output = 0.0f;
  pre_error = 0.0f;
}

void Class_PID::calculate(float Measure, float Target) {
  float pout = 0.0f;
  float iout = 0.0f;
  float dout = 0.0f;
  float fout = 0.0f;
  float error;
  float abs_error;

  target = Target;
  measure = Measure;

  error = target - measure;
  abs_error = ABS(error);

  if (abs_error < deadband) {
    target = measure;
    error = 0.0f;
    abs_error = 0.0f;
  }
  pout = Kp * error;

  iout += Ki * error * dt;

  dout = Kd * (error - pre_error) / dt;

  fout = Kf * (target - pre_target);

  output = pout + iout + dout + fout;
  output = LIMIT_MAX_MIN(output, outMax, -outMax);

  pre_error = error;
  pre_target = target;
  pre_measure = measure;
  pre_output = output;
}