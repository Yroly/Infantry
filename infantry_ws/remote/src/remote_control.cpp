#include "remote_control.hpp"
#include "bsp_rc.hpp"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t rc_ctrl;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
