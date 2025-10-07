#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "init_task.hpp"
#include "cmsis_os.h"
#include "tim.h"

void Init_Task();
extern TaskHandle_t Boozer_Task_handle;
extern void Init_Task();
void Boozer_Task();

#ifdef __cplusplus
}
#endif
