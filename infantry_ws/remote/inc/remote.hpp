#pragma once
#include <stdint.h>

struct Remote {
  int16_t ch0 = 0;
  int16_t ch1 = 0;
  int16_t ch2 = 0;
  int16_t ch3 = 0;
  int8_t s1 = 0;
  int8_t s2 = 0;
};

struct Mouse {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int16_t press_l = 0;
  int16_t press_r = 0;
  int16_t last_press_l = 0;
  int16_t last_press_r = 0;
};

struct Key {
  uint16_t W : 1;
  uint16_t S : 1;
  uint16_t A : 1;
  uint16_t D : 1;
  uint16_t Shift : 1;
  uint16_t Ctrl : 1;
  uint16_t Q : 1;
  uint16_t E : 1;
  uint16_t R : 1;
  uint16_t F : 1;
  uint16_t G : 1;
  uint16_t Z : 1;
  uint16_t X : 1;
  uint16_t C : 1;
  uint16_t V : 1;
  uint16_t B : 1;

  Key()
      : W(0), S(0), A(0), D(0), Shift(0), Ctrl(0), Q(0), E(0), R(0), F(0), G(0),
        Z(0), X(0), C(0), V(0), B(0) {}
};

enum class InputMode : uint8_t {
  REMOTE_INPUT = 1,
  STOP = 2,
  KEY_MOUSE_INPUT = 3
};

class RemoteControl {
public:
  static constexpr int STICK_OFFSET = 1024;

  Remote rc;
  Mouse mouse;
  Key key;
  Key lastkey;
  InputMode mode = InputMode::STOP;

  float Key_ch[4] = {0};
  float Mouse_ch[3] = {0};

  RemoteControl() = default;

  /**
   * @brief 接收遥控器串口原始数据
   * @param[in] RxMsg 遥控器串口原始数据
   */
  void receiveData(unsigned char *RxMsg);

  /**
   * @brief 清空遥控器数据
   */
  void clear();

  /**
   * @brief 遥控器输入模式处理
   */
  void processRemoteInput(Remote *rc);

  /**
   * @brief 键鼠输入模式处理
   */
  void processKeyMouseInput(Mouse *mouse, Key key, Key lastkey);

  /**
   * @brief 急停模式处理
   */
  void processStop();
  /**
   * @brief 去除死区
   */
  void deadline_limit(float value, float dealine);

  static void Remote_Rx(void *pvParameters) {
    RemoteControl *remoteInstance = static_cast<RemoteControl *>(pvParameters);
    if (remoteInstance) {
      unsigned char *RxMsg =
          nullptr; // Replace nullptr with actual data source as needed
      remoteInstance->receiveData(RxMsg);
    }
  }
};
