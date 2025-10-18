#pragma once

#include "buzzer.hpp"
#include "bsp_buzzer.hpp"
#include "cmsis_os.h"
#include "init_task.hpp"

void Buzzer_Task(void const *pvParameters);
