#include "uartCmd.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

// 缓冲区定义
uint8_t uartCmdBuffer_send[UARTCMD_BUFFER_SEND_SIZE];
uint8_t uartCmdBuffer_rcv[UARTCMD_BUFFER_RCV_SIZE];
uint16_t uartCmd_byte_counts = 0; // 收到的字节数

// 命令表
static const uart_cmd_t *cmd_table[16];
static uint8_t cmd_count = 0;

// 示例变量
static int val1 = 0;

// 发送缓冲区
static char send_buffer[UARTCMD_BUFFER_SEND_SIZE];

/**
 * @brief 初始化命令系统
 */
void uart_cmd_init(void)
{
    cmd_count = 0;
    
    // 注册示例命令
    static const uart_cmd_t example_cmds[] = {
        {"test1", cmd_test1, "测试命令1"},
        {"test2", cmd_test2, "测试命令2"},
        {"set val1", cmd_set_val1, "设置val1的值，例如: set val1=100 或 set val1 = 200"},
        {"get val1", cmd_get_val1, "获取val1的值"},
    };
    
    for (int i = 0; i < sizeof(example_cmds) / sizeof(example_cmds[0]); i++) {
        uart_cmd_register(&example_cmds[i]);
    }
}

/**
 * @brief 注册命令
 */
void uart_cmd_register(const uart_cmd_t *cmd)
{
    if (cmd_count < sizeof(cmd_table) / sizeof(cmd_table[0])) {
        cmd_table[cmd_count++] = cmd;
    }
}

/**
 * @brief 解析整数
 */
int uart_cmd_parse_int(const char *str, int *value)
{
    if (str == NULL || value == NULL) {
        return -1;
    }
    
    char *endptr;
    long val = strtol(str, &endptr, 10);
    
    if (endptr == str) {
        return -1; // 没有数字
    }
    
    *value = (int)val;
    return 0;
}

/**
 * @brief 从字符串中提取数字
 * 支持格式: "=100", " = 100", "= 100", " =100"
 */
int uart_cmd_extract_number(const char *str, int *value)
{
    if (str == NULL || value == NULL) {
        return -1;
    }
    
    // 跳过空格
    while (*str == ' ') str++;
    
    // 查找等号
    const char *equal_sign = strchr(str, '=');
    if (equal_sign == NULL) {
        return -1; // 没有等号
    }
    
    // 跳过等号和可能的空格
    const char *num_start = equal_sign + 1;
    while (*num_start == ' ') num_start++;
    
    // 解析数字
    return uart_cmd_parse_int(num_start, value);
}

/**
 * @brief 处理接收到的数据
 */
void uart_cmd_process(const uint8_t *data, uint16_t length)
{
    if (length < 2) {
        return; // 数据太短
    }
    
    // 验证CRC
    if (crc16_verify(data, length) != 0) {
        uart_cmd_send_response("CRC校验失败");
        return;
    }
    
    // 提取命令数据（去掉CRC字节）
    uint16_t cmd_length = length - 2;
    char cmd_buffer[UARTCMD_MAX_CMD_LENGTH + 1];
    
    if (cmd_length > UARTCMD_MAX_CMD_LENGTH) {
        cmd_length = UARTCMD_MAX_CMD_LENGTH;
    }
    
    // 复制命令数据并添加字符串结束符
    memcpy(cmd_buffer, data, cmd_length);
    cmd_buffer[cmd_length] = '\0';
    
    // 查找匹配的命令
    for (int i = 0; i < cmd_count; i++) {
        const uart_cmd_t *cmd = cmd_table[i];
        size_t cmd_len = strlen(cmd->cmd);
        
        // 检查命令是否匹配
        if (strncmp(cmd_buffer, cmd->cmd, cmd_len) == 0) {
            // 提取参数
            const char *args = NULL;
            if (cmd_length > cmd_len) {
                args = cmd_buffer + cmd_len;
                // 跳过参数前的空格
                while (*args == ' ') args++;
            }
            
            // 执行命令处理函数
            cmd->handler(args);
            return;
        }
    }
    
    // 没有找到匹配的命令
    uart_cmd_send_response("未知命令: %s", cmd_buffer);
}

/**
 * @brief 发送响应（类似printf功能）
 */
void uart_cmd_send_response(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    
    // 格式化字符串
    int len = vsnprintf(send_buffer, sizeof(send_buffer) - 3, format, args);
    va_end(args);
    
    if (len > 0) {
        // 添加换行符
        if (len < sizeof(send_buffer) - 2) {
            send_buffer[len] = '\r';
            send_buffer[len + 1] = '\n';
            send_buffer[len + 2] = '\0';
            len += 2;
        }
        
        // 计算CRC
        uint16_t crc = crc16_ccitt((uint8_t *)send_buffer, len);
        
        // 发送数据
        uart_cmd_send_data((uint8_t *)send_buffer, len);
        
        // 发送CRC（小端字节序）
        uint8_t crc_bytes[2] = {crc & 0xFF, (crc >> 8) & 0xFF};
        uartCmd_send(crc_bytes, 2);
    }
}

/**
 * @brief 发送数据（带CRC）
 */
void uart_cmd_send_data(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return;
    }
    
    // 计算CRC
    uint16_t crc = crc16_ccitt(data, length);
    
    // 发送数据
    uartCmd_send((uint8_t *)data, length);
    
    // 发送CRC（小端字节序）
    uint8_t crc_bytes[2] = {crc & 0xFF, (crc >> 8) & 0xFF};
    uartCmd_send(crc_bytes, 2);
}

/**
 * @brief 发送完成后调用的函数
 */
void uart_send_finish_handle()
{
    // 可以在这里添加发送完成后的处理逻辑
}

/**
 * @brief 接收完成后调用的函数
 * 当设备收到连续的几个字节数据，后超过一段时间没收到数据，就是接收完成
 */
void uart_recive_finish_handle()
{
    uartCmd_byte_counts = UARTCMD_BUFFER_RCV_SIZE - uartCmd_rcv_nums();
    
    // 处理接收到的命令
    if (uartCmd_byte_counts > 0) {
        uart_cmd_process(uartCmdBuffer_rcv, uartCmd_byte_counts);
    }
}

// ==================== 示例命令处理函数 ====================

/**
 * @brief 测试命令1
 */
void cmd_test1(const char *args)
{
    (void)args; // 未使用参数
    uart_cmd_send_response("测试命令1执行成功");
}

/**
 * @brief 测试命令2
 */
void cmd_test2(const char *args)
{
    (void)args; // 未使用参数
    uart_cmd_send_response("测试命令2执行成功");
}

/**
 * @brief 设置val1的值
 */
void cmd_set_val1(const char *args)
{
    if (args == NULL || *args == '\0') {
        uart_cmd_send_response("错误: 需要指定值，例如: set val1=100");
        return;
    }
    
    int new_value;
    if (uart_cmd_extract_number(args, &new_value) == 0) {
        val1 = new_value;
        uart_cmd_send_response("设置成功: val1 = %d", val1);
    } else {
        uart_cmd_send_response("错误: 无法解析数值");
    }
}

/**
 * @brief 获取val1的值
 */
void cmd_get_val1(const char *args)
{
    (void)args; // 未使用参数
    uart_cmd_send_response("val1 = %d", val1);
}
