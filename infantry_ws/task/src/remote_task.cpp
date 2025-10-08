#include "remote_task.hpp"
#include "remote.hpp"
#include "variate.hpp"

RemoteTask remotetask;
DeviceStates DeviceState;
eSystemState SystemState;

void RemoteTask::taskLoop() {
  static portTickType currentTime;
  static uint8_t cnt = 0;

  for (;;) {
    currentTime = xTaskGetTickCount();

    if (DeviceState.Remote_State != Device_Online) {
      suspendTasks();
      rcControl.clear();
      clearKeyChannel();
      SystemState = SYSTEM_STARTING;

    } else {
      resumeTasks();
    }
    updateGimbalAction();

    switch (cnt++) {
      //   case 0:CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t
      //   *)&Gimbal_action);break; case 1:CAN_Send_StdDataFrame(&hcan2, 0x130,
      //   (uint8_t *)&Gimbal_data);cnt = 0;break;
    }

    // WatchDog_Polling();
    vTaskDelayUntil(&currentTime, 15);
  }
}

void RemoteControl::processRemoteInput(Remote *rc) {
  mode = InputMode::REMOTE_INPUT;

  Key_ch[0] = (float)(rc->ch0 - RemoteControl::STICK_OFFSET) / 660.0f;
  Key_ch[1] = (float)(rc->ch1 - RemoteControl::STICK_OFFSET) / 660.0f;
  Key_ch[2] = (float)(rc->ch2 - RemoteControl::STICK_OFFSET) / 660.0f;
  Key_ch[3] = (float)(rc->ch3 - RemoteControl::STICK_OFFSET) / 660.0f;

  RemoteControl::deadline_limit(Key_ch[0], 0.1f);
  RemoteControl::deadline_limit(Key_ch[1], 0.1f);
  RemoteControl::deadline_limit(Key_ch[2], 0.1f);
  RemoteControl::deadline_limit(Key_ch[3], 0.1f);
}
void RemoteControl::processStop() { mode = InputMode::STOP; }
void RemoteControl::processKeyMouseInput(Mouse *mouse, Key key, Key lastkey) {
  mode = InputMode::KEY_MOUSE_INPUT;
  limit(mouse->x, 200, -200);
  limit(mouse->y, 200, -200);
  limit(mouse->z, 200, -200);

  Mouse_ch[0] = (float)(mouse->x) / 200;
  Mouse_ch[1] = (float)(mouse->y) / 200;
  Mouse_ch[2] = (float)(mouse->z) / 200;

  deadline_limit(Mouse_ch[0], 0.01f);
  deadline_limit(Mouse_ch[1], 0.01f);
  deadline_limit(Mouse_ch[2], 0.01f);
}

void RemoteTask::suspendTasks() {
  // osThreadSuspend(Chassis_Task_handle);
  // osThreadSuspend(Gimbal_Task_handle);
  // osThreadSuspend(Shoot_Task_handle);
}
void RemoteTask::resumeTasks() {
  // osThreadResume(Chassis_Task_handle);
  // osThreadResume(Gimbal_Task_handle);
  // osThreadResume(Shoot_Task_handle);
}
void RemoteTask::clearKeyChannel() {
  for (int i = 0; i < 4; ++i)
    rcControl.Key_ch[i] = 0;
}

void RemoteTask::updateGimbalAction() {
  if (rcControl.key.Z) {
    // Gimbal_action.Key = 1;
  } else if (rcControl.key.Ctrl) {
    // Gimbal_action.Key = 2;
  } else {
    // Gimbal_action.Key = 0;
  }
}