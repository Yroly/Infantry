#include "ins_task.hpp"
#include "BMI088driver.hpp"
#include "QuaternionEKF.hpp"
#include "bsp_PWM.hpp"
#include "controller.hpp"

IMU_t IMU;
INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};
// extern WatchDog_TypeDef IMU_Dog;
void IMU_Rx();
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 45.0;
void InitQuaternion(float *init_q4);

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3],
                                 float accel[3]);
void Cross3d(float *v1, float *v2, float *res);
float Dot3d(float *v1, float *v2);
float NormOf3d(float *v);
float *Norm3d(float *v);

void INS_Init(void) {
  IMU_Param.scale[0] = 1;
  IMU_Param.scale[1] = 1;
  IMU_Param.scale[2] = 1;
  IMU_Param.Yaw = 0;
  IMU_Param.Pitch = 0;
  IMU_Param.Roll = 0;
  IMU_Param.flag = 1;

  //	  RefTemp = BMI088.Temperature;
  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

  // imu heat init
  PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  INS.AccelLPF = 0.0085;
}

void INS_Task(void) {

  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.81f};
  dt = DWT_GetDeltaT(&INS_DWT_Count);
  t += dt;

  // ins update
  if ((count % 1) == 0) {
    BMI088_Read(&BMI088);

    INS.Accel[0] = BMI088.Accel[0];
    INS.Accel[1] = BMI088.Accel[1];
    INS.Accel[2] = BMI088.Accel[2];
    INS.Gyro[0] = BMI088.Gyro[0];
    INS.Gyro[1] = BMI088.Gyro[1];
    INS.Gyro[2] = BMI088.Gyro[2];

    // demo function,用于修正安装误差,可以不管,本demo暂时没用
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    INS.atanxz = -atan2f(INS.Accel[0], INS.Accel[2]) * 180 / PI;
    INS.atanyz = atan2f(INS.Accel[1], INS.Accel[2]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2],
                             INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] =
          (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
          INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n,
                          INS.q); // 转换回导航系n

    // 获取最终数据
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    IMU_Rx();

    // Feed_Dog(&IMU_Dog);
  }

  // temperature control
  if ((count % 2) == 0) {
    // 500hz
    IMU_Temperature_Ctrl();
  }

  if ((count % 1000) == 0) {
    // 200hz
  }

  count++;
}
// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
void InitQuaternion(float *init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
  float axis_rot[3] = {0};           // 旋转轴
  // 读取100次加速度计数据,取平均值作为初始值
  for (uint8_t i = 0; i < 100; ++i) {
    BMI088_Read(&BMI088);
    acc_init[0] += BMI088.Accel[0];
    acc_init[1] += BMI088.Accel[1];
    acc_init[2] += BMI088.Accel[2];
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i)
    acc_init[i] /= 100;
  Norm3d(acc_init);
  // 计算原始加速度矢量和导航系重力加速度矢量的夹角
  float angle = acosf(Dot3d(acc_init, gravity_norm));
  Cross3d(acc_init, gravity_norm, axis_rot);
  Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i)
    init_q4[i + 1] =
        axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

void IMU_Rx() {
  IMU.Angle_Pitch = INS.Pitch;
  IMU.Angle_Yaw = INS.Yaw;
  IMU.Angle_Roll = INS.Roll;
  IMU.Angle_Yawcontinuous = INS.YawTotalAngle;
  IMU.Gyro_Pitch = INS.Gyro[0];
  IMU.Gyro_Roll = INS.Gyro[1];
  IMU.Gyro_Yaw = INS.Gyro[2];
  //	   float diff = QEKF_INS.Yaw - QEKF_INS.YawAngleLast;
  //		 if (diff > 100)
  //			IMU.r--;
  //		 else if (diff < -100)
  IMU.r = QEKF_INS.YawRoundCount;
  for (int i = 0; i < 4; i++)
    IMU.q[i] = INS.q[i];
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                     (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                     (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                     (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                     (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                     (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                     (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief
 * reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3],
                                 float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
      fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[0] = c_11 * gyro_temp[0] + c_12 * gyro_temp[1] + c_13 * gyro_temp[2];
  gyro[1] = c_21 * gyro_temp[0] + c_22 * gyro_temp[1] + c_23 * gyro_temp[2];
  gyro[2] = c_31 * gyro_temp[0] + c_32 * gyro_temp[1] + c_33 * gyro_temp[2];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    accel_temp[i] = accel[i];

  accel[0] = c_11 * accel_temp[0] + c_12 * accel_temp[1] + c_13 * accel_temp[2];
  accel[1] = c_21 * accel_temp[0] + c_22 * accel_temp[1] + c_23 * accel_temp[2];
  accel[2] = c_31 * accel_temp[0] + c_32 * accel_temp[1] + c_33 * accel_temp[2];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 *
 */
void IMU_Temperature_Ctrl(void) {
  PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

  TIM_Set_PWM(&htim10, TIM_CHANNEL_1,
              float_constrain(float_rounding(TempCtrl.Output), 0, UINT16_MAX));
}

//------------------------------------functions below are not used in this
//demo-------------------------------------------------
//----------------------------------you can read them for learning or
//programming-----------------------------------------------
//----------------------------------they could also be helpful for further
//design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt) {
  float qa, qb, qc;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll) {
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
         57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
           57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}
// 三维向量归一化
float *Norm3d(float *v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 计算模长
float NormOf3d(float *v) {
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len) {
  float sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    buf[i] = buf[i + 1];
    sum += buf[i];
  }
  buf[len - 1] = new_data;
  sum += new_data;
  return sum / len;
}

// void MatInit(mat *m, uint8_t row, uint8_t col)
//{
//     m->numCols = col;
//     m->numRows = row;
//     m->pData = (float *)zmalloc(row * col * sizeof(float));
// }

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q) {
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = arm_cos_f32(Pitch / 2);
  cosYaw = arm_cos_f32(Yaw / 2);
  cosRoll = arm_cos_f32(Roll / 2);
  sinPitch = arm_sin_f32(Pitch / 2);
  sinYaw = arm_sin_f32(Yaw / 2);
  sinRoll = arm_sin_f32(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
