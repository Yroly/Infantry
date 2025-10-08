#include "init_task.hpp"

void Init_Task() {
  taskENTER_CRITICAL();

  HAL_TIM_Base_Start_IT(&htim3);

  xTaskCreate((TaskFunction_t)Boozer_Task, "Boozer_Task", 256, NULL, 4,
              &Boozer_Task_handle);

  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}