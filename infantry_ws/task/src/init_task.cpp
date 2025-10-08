#include "init_task.hpp"
#include "cmsis_os.h"

void Init_Task() {
  taskENTER_CRITICAL();

  

  HAL_TIM_Base_Start_IT(&htim3);

  // xTaskCreate((TaskFunction_t)RemoteTask::taskWrapper,
  //             "RemoteTask::taskWrapper", 256, NULL, 7, &Remote_Task_handle);
  xTaskCreate((TaskFunction_t)Boozer_Task, "Boozer_Task", 256, NULL, 4,
              &Boozer_Task_handle);

  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}