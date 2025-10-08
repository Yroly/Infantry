#pragma once

#include "stdint.h"

#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX

typedef enum{
	SYSTEM_STARTING = 0,
	SYSTEM_RUNNING  = 1,
} eSystemState;
typedef enum{
	Device_Offline = 0,
	Device_Online  = 1,
	Device_Error   = 2	   
} eDeviceState;
typedef struct {
	// eDeviceState Remote_State, IMU_State, Gimbal_State[GIMBAL_SUM], Shoot_State[FRIC_SUM], Pluck_State, Down_State, PC_State,Referee_State;
    eDeviceState Remote_State;
}DeviceStates;

extern DeviceStates DeviceState;
extern eSystemState SystemState;