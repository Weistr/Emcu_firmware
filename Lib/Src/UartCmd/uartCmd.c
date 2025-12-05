#include "uartCmd.h"
#include "main.h"
uint8_t uartCmdBuffer_send[uartCmdBuffer_send_size];
uint8_t uartCmdBuffer_rcv[uartCmdBuffer_rcv_size];
uint16_t uartCmd_byte_counts = 0;//收到的字节数


//发送完后会调用该函数
void uart_send_finish_handle()
{

}
//接收完数据后会调用该函数
//当设备收到连续的几个字节数据，后超过一段时间没收到数据，就是接收完成
void uart_recive_finish_handle()
{
    uartCmd_byte_counts = uartCmdBuffer_rcv_size - uartCmd_rcv_nums();
    //uartCmd_send(uartCmdBuffer_rcv,uartCmd_byte_counts);//测试用


}