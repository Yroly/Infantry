#include "remote.hpp"
void RemoteControl::clear() {
  rc.ch0 = STICK_OFFSET;
  rc.ch1 = STICK_OFFSET;
  rc.ch2 = STICK_OFFSET;
  rc.ch3 = STICK_OFFSET;
  rc.s1 = 2;
  rc.s2 = 2;

  mouse.x = 0;
  mouse.y = 0;
  mouse.z = 0;
  mouse.press_l = 0;
  mouse.press_r = 0;
  mouse.last_press_l = 0;
  mouse.last_press_r = 0;

  key = Key();
  lastkey = Key();
}

void RemoteControl::receiveData(unsigned char *RxMsg) {
  rc.ch0 = (RxMsg[0] | (RxMsg[1] << 8)) & 0x07FF;
  rc.ch1 = ((RxMsg[1] >> 3) | (RxMsg[2] << 5)) & 0x07FF;
  rc.ch2 = ((RxMsg[2] >> 6) | (RxMsg[3] << 2) | (RxMsg[4] << 10)) & 0x07FF;
  rc.ch3 = ((RxMsg[4] >> 1) | (RxMsg[5] << 7)) & 0x07FF;

  rc.s1 = (RxMsg[5] >> 4 & 0x000C) >> 2;
  rc.s2 = (RxMsg[5] >> 4 & 0x0003);

  mouse.x = static_cast<int16_t>(RxMsg[6] | (RxMsg[7] << 8));
  mouse.y = static_cast<int16_t>(RxMsg[8] | (RxMsg[9] << 8));
  mouse.z = static_cast<int16_t>(RxMsg[10] | (RxMsg[11] << 8));

  mouse.press_l = RxMsg[12];
  mouse.press_r = RxMsg[13];

  *(uint16_t *)&(key) = RxMsg[14] | RxMsg[15] << 8;

  // 将16位键盘数据拷贝到 key
  // uint16_t keyData = static_cast<uint16_t>(RxMsg[14] | (RxMsg[15] << 8));
  // memcpy(&key, &keyData, sizeof(Key));

  // 根据 s2 选择模式
  switch (static_cast<InputMode>(rc.s2)) {
  case InputMode::REMOTE_INPUT:
    processRemoteInput(&rc);
    break;
  case InputMode::KEY_MOUSE_INPUT:
    processKeyMouseInput(&mouse, key, lastkey);
    break;
  case InputMode::STOP:
    processStop();
    break;
  }

  mouse.last_press_l = mouse.press_l;
  mouse.last_press_r = mouse.press_r;
  lastkey = key;
}

void RemoteControl::deadline_limit(float value, float deadline) {
  if (value >= -deadline && value <= deadline) {
    value = 0;
  }
} 