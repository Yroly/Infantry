#pragma once

#include "cmsis_os.h"
#include "init_task.hpp"
#include "remote.hpp"


class RemoteTask {
public:
  RemoteTask() = default;

  void taskLoop();
  static void taskWrapper(void* pvParameters){
    RemoteTask* taskInstance = static_cast<RemoteTask*>(pvParameters);
    if(taskInstance){
        taskInstance->taskLoop();
    }
  }
  RemoteControl rcControl;

private:
  void suspendTasks();
  void resumeTasks();
  void clearKeyChannel();
  void updateGimbalAction();
};
