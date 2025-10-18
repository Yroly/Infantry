#include "init_task.hpp"
#include "cmsis_os.h"
#include "remote_control.hpp"

void Init_Task() {
  taskENTER_CRITICAL();

  remote_control_init();

  HAL_TIM_Base_Start_IT(&htim3);

  // xTaskCreate((TaskFunction_t)RemoteTask::taskWrapper,
  //             "RemoteTask::taskWrapper", 256, NULL, 7, &Remote_Task_handle);
  xTaskCreate((TaskFunction_t)Buzzer_Task, "Buzzer_Task", 256, NULL, 4,
              &Buzzer_Task_handle);

  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}