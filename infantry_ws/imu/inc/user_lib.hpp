/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    float input;        //ÊäÈëÊý¾Ý
    float out;          //Êä³öÊý¾Ý
    float min_value;    //ÏÞ·ù×îÐ¡Öµ
    float max_value;    //ÏÞ·ù×î´óÖµ
    float frame_period; //Ê±¼ä¼ä¸ô
} ramp_function_source_t;

typedef  struct __attribute__((packed))
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

//¿ìËÙ¿ª·½
float Sqrt(float x);

//Ð±²¨º¯Êý³õÊ¼»¯
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
//Ð±²¨º¯Êý¼ÆËã
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//¾ø¶ÔÏÞÖÆ
float abs_limit(float num, float Limit);
//ÅÐ¶Ï·ûºÅÎ»
float sign(float value);
//¸¡µãËÀÇø
float float_deadband(float Value, float minValue, float maxValue);
// int26ËÀÇø
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//ÏÞ·ùº¯Êý
float float_constrain(float Value, float minValue, float maxValue);
//ÏÞ·ùº¯Êý
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//Ñ­»·ÏÞ·ùº¯Êý
float loop_float_constrain(float Input, float minValue, float maxValue);
//½Ç¶È ¡ãÏÞ·ù 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

//»¡¶È¸ñÊ½»¯Îª-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

#endif
