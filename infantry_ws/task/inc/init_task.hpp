#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "tim.h"
#include "boozer_task.hpp"
#include "remote_task.hpp"

extern TaskHandle_t Remote_Task_handle;
extern TaskHandle_t Boozer_Task_handle;

void Init_Task();
void Boozer_Task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
