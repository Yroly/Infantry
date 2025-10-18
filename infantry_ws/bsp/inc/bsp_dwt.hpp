#pragma once
#ifdef __cplusplus
extern "C"{
#endif
#include "main.h"
#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

extern void DWT_Init(uint32_t CPU_Freq_mHz);

extern DWT_Time_t SysTime;

#ifdef __cplusplus
}
#endif

extern float DWT_GetDeltaT(uint32_t *cnt_last);
extern double DWT_GetDeltaT64(uint32_t *cnt_last);
extern float DWT_GetTimeline_s(void);
extern float DWT_GetTimeline_ms(void);
extern uint64_t DWT_GetTimeline_us(void);
extern void DWT_Delay(float Delay);
extern void DWT_SysTimeUpdate(void);