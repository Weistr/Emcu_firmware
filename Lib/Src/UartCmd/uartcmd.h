#ifndef _UARTCMD_H
#define _UARTCMD_H
#include "gd32e23x_usart.h"
#define uartCmdBuffer_send_size 8
#define uartCmdBuffer_rcv_size 8
//发送函数API,需要在其他地方实现
extern void uartCmd_send(uint8_t* chr,uint16_t num);
//读取接收数据收到数据的字节数,需要在其他地方实现
extern uint16_t uartCmd_rcv_nums(void);
#endif // !_UARTCMD_H