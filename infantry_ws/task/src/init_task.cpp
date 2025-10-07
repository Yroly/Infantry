#include "init_task.hpp"
#include "stm32f4xx_hal_gpio.h"

void Init_Task() {
  taskENTER_CRITICAL();

  HAL_TIM_Base_Start_IT(&htim3);

  xTaskCreate((TaskFunction_t)Boozer_Task_handle, "Boozer_Task", 256, NULL, 4,
              &Boozer_Task_handle);

  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}