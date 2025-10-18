#pragma once

#include "main.h"

//-----------------------------------------
// 云台控制类 Gimbal
//-----------------------------------------
class Gimbal {
public:
    //-------------------------------------
    // 枚举类型定义
    //-------------------------------------
    enum class PidMode : uint8_t {
        INIT = 0,
        MECH,
        GYRO,
        AIM,
        MODE_SUM
    };

    enum class CtrlMode : uint8_t {
        Normal = 0,
        Aim,
        Follow,
        Test
    };

    enum class Mode : uint8_t {
        Gyro = 0,
        Mech,
        ModeSum
    };

    //-------------------------------------
    // 结构体定义
    //-------------------------------------
    struct PTZAngleRef {
        float Pitch = 0.0f;
        float Yaw   = 0.0f;
    };

    struct Angle {
        float Pitch = 0.0f;
        float Roll  = 0.0f;
        float Yaw   = 0.0f;
        int16_t r   = 0;
        float Last  = 0.0f;
        float ContinuousYaw = 0.0f;
    };

    struct Speed {
        float Yaw   = 0.0f;
        float Pitch = 0.0f;
    };

    //-------------------------------------
    // 成员变量
    //-------------------------------------
    PidMode pidMode;
    CtrlMode ctrlMode;

    int lastCtrl = 0;
    int initFlag = 1;

    PTZAngleRef ref[static_cast<int>(Mode::ModeSum)];
    Angle angle[static_cast<int>(Mode::ModeSum)];
    Speed speed[static_cast<int>(Mode::ModeSum)];

    float yawInit   = 0.0f;
    float pitchInit = 0.0f;
    // float increase[GIMBAL_SUM] = {0.0f};
    // uint8_t midMode = 0;

    // // PID 控制器
    // PID_Smis placePIDs[GIMBAL_SUM][static_cast<int>(PidMode::MODE_SUM)];
    // PID speedPIDs[GIMBAL_SUM][static_cast<int>(PidMode::MODE_SUM)];

    // PID_TypeDef speedPidPitch[static_cast<int>(PidMode::MODE_SUM)];
    // PID_TypeDef placePidPitch[static_cast<int>(PidMode::MODE_SUM)];
    // PID_TypeDef speedPidYaw[static_cast<int>(PidMode::MODE_SUM)];
    // PID_TypeDef placePidYaw[static_cast<int>(PidMode::MODE_SUM)];

    // 前馈项
    float Kff_v = 0.0f;
    float Kff_a = 0.0f;

    //-------------------------------------
    // 构造与初始化
    //-------------------------------------
    Gimbal() = default;
    void init();
    void medianInit();

    //-------------------------------------
    // 控制与更新函数
    //-------------------------------------
    void rcControl();
    void keyControl();
    void stop();

    void decideCtrl();
    void updateRef();
    void updateReal();
    void pidCompute();

    //-------------------------------------
    // 数据发送函数
    //-------------------------------------
    void send();
    void sendDown();

private:
    // 内部辅助函数
    float limit(float value, float min, float max);
};

