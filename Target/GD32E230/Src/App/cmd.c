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
