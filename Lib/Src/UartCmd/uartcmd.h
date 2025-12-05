#ifndef _UARTCMD_H
#define _UARTCMD_H

#include <stdint.h>
#include <stddef.h>
#include "crc.h"
// gd32e23x_usart.h 包含在需要的地方，这里只声明需要的类型

// 缓冲区大小
#define UARTCMD_BUFFER_SEND_SIZE 128
#define UARTCMD_BUFFER_RCV_SIZE  128
#define UARTCMD_MAX_CMD_LENGTH   64
#define UARTCMD_MAX_ARGS         8

// 命令处理函数类型
typedef void (*cmd_handler_t)(const char *args);

// 命令结构体
typedef struct {
    const char *cmd;          // 命令字符串
    cmd_handler_t handler;    // 处理函数
    const char *description;  // 命令描述
} uart_cmd_t;

// 外部变量声明
extern uint8_t uartCmdBuffer_send[UARTCMD_BUFFER_SEND_SIZE];
extern uint8_t uartCmdBuffer_rcv[UARTCMD_BUFFER_RCV_SIZE];
extern uint16_t uartCmd_byte_counts;

// 发送函数API,需要在其他地方实现
extern void uartCmd_send(uint8_t* chr, uint16_t num);

// 读取接收数据收到数据的字节数,需要在其他地方实现
extern uint16_t uartCmd_rcv_nums(void);

// 新增函数声明
void uart_cmd_init(void);
void uart_cmd_register(const uart_cmd_t *cmd);
void uart_cmd_process(const uint8_t *data, uint16_t length);
void uart_cmd_send_response(const char *format, ...);
void uart_cmd_send_data(const uint8_t *data, uint16_t length);
int uart_cmd_parse_int(const char *str, int *value);
int uart_cmd_extract_number(const char *str, int *value);

// 示例命令处理函数
void cmd_test1(const char *args);
void cmd_test2(const char *args);
void cmd_set_val1(const char *args);
void cmd_get_val1(const char *args);

#endif // _UARTCMD_H
