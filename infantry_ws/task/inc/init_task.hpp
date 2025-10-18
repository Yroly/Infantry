#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "tim.h"
#include "buzzer_task.hpp"
#include "remote_task.hpp"

extern TaskHandle_t Remote_Task_handle;
extern TaskHandle_t Buzzer_Task_handle;

void Init_Task();
void Buzzer_Task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
