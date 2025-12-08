# UART 命令系统使用指南

## 概述

UART 命令系统是一个轻量级、可扩展的串口命令处理框架，专为嵌入式系统设计。它提供了类似 shell 的命令行接口，支持命令注册、参数解析、数据校验和格式化输出等功能。



### 1. 文件配置

首先要在外部完成函数的调用
```c
//发送完后该函数会被调用
void uart_send_finish_handle(void)
//接收完后该函数会被调用, 缓冲器数据会被丢弃
void uart_recive_finish_handle(void)
```
确保收到数据后，要存储在uartCmdBuffer_rcv缓冲区

然后要在外部实现函数
```c
//发送数据
void uartCmd_send(uint8_t* chr,uint16_t num)
//读取收到的数据的字节数
uint16_t uartCmd_rcv_nums(void)
```

接下来需要修改缓冲区大小，路径Lib\Src\UartCmd\uartcmd.h
```c
// 缓冲区大小
#define UARTCMD_BUFFER_SEND_SIZE 128
#define UARTCMD_BUFFER_RCV_SIZE  128
```

设置校验模式
在 `uartcmd.h` 中定义校验模式：

```c
// 校验模式选择
// 定义以下其中一个宏来选择校验模式：
// #define UARTCMD_CHECK_MODE_CRC       // CRC16校验
// #define UARTCMD_CHECK_MODE_CHECKSUM  // 校验和（前n-1字节相加取低8位）
 #define UARTCMD_CHECK_MODE_NONE      // 无校验
```
注册命令

系统默认注册了以下示例命令：

| 命令 | 功能 | 示例 |
|------|------|------|
| `test1` | 测试命令1 | `test1` |
| `set val1` | 设置变量 val1 | `set val1=100` 或 `set val1 = 100` |
| `get val1` | 获取变量 val1 | `get val1` |

用户需要再外部定义命令和实现函数，下面是例子
```c
#include "uartCmd.h"

// 用户命令处理函数声明
void cmd_userFun1(const char *args);
void cmd_set_userval1(const char *args);
void cmd_get_userval1(const char *args);

// 用户命令数组
const uart_cmd_t user_cmds[] = {
    {"userFun1", cmd_userFun1},
    {"set userval1", cmd_set_userval1},
    {"get userval1", cmd_get_userval1},
};

// 用户命令数组大小
uint16_t user_cmds_count = sizeof(user_cmds) / sizeof(user_cmds[0]);
//uint16_t user_cmds_count = 100;
// 用户命令处理函数实现
void cmd_userFun1(const char *args)
{
    (void)args; // 未使用参数
    // 在这里实现 userFun1 的功能
    // 可以使用 uart_cmd_send_response 发送响应
    uart_cmd_send_response("User function1 executed");
}
// 示例：解析数字
int userval1;
void cmd_set_userval1(const char *args)
{
    if (args == NULL || *args == '\0') {
        uart_cmd_send_response("Error: Need to specify value, e.g.: set userval1=100");
        return;
    }
    
    // 在这里解析参数并设置 userval1 的值

    if (uart_cmd_extract_number(args, &userval1) == 0) {
        // 设置 userval1 的值（需要定义变量）
        // userval1 = value;
        uart_cmd_send_response("Set userval1 = %d", userval1);
    } else {
        uart_cmd_send_response("Error: Cannot parse value");
    }
}

void cmd_get_userval1(const char *args)
{
    // get 命令不应该有参数
    if (args != NULL && *args != '\0') {
        // 检查参数是否包含 '='（错误语法，如 "get userval1 = 100"）
        const char *ptr = args;
        while (*ptr != '\0') {
            if (*ptr == '=') {
                uart_cmd_send_response("Error: 'get userval1' command does not take arguments with '='");
                return;
            }
            ptr++;
        }
        
        // 如果有其他参数（如 "get userval1 s"），也返回错误
        uart_cmd_send_response("Error: 'get userval1' command does not take arguments");
        return;
    }
    
    // 在这里获取 userval1 的值并发送响应
     uart_cmd_send_response("userval1 = %d", userval1);
}

```
## 其他说明
未找到匹配指令会调用该函数
```c
//未找到匹配指令
void unkonwnCmd()
{

} 
```
## 版本历史

- v1.0.0 (2025-12-06)
  - 初始版本
  - 支持命令注册和参数解析
  - 三种校验模式
  - 用户自定义命令支持

## 许可证

本项目使用 MIT 许可证。详见项目根目录的 LICENSE 文件。
