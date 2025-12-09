#include "uartCmd.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>


// 校验和计算函数
static uint8_t calculate_checksum(const uint8_t *data, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// 验证校验和
static uint8_t verify_checksum(const uint8_t *data, uint16_t length)
{
    if (length < 1) {
        return 1; // 数据长度不足
    }
    
    // 计算前length-1字节的校验和
    uint8_t calculated_checksum = calculate_checksum(data, length - 1);
    
    // 比较校验和
    return (calculated_checksum == data[length - 1]) ? 0 : 1;
}

// 通用数据验证函数（根据选择的校验模式）
static uint8_t verify_data(const uint8_t *data, uint16_t length)
{
#if defined(UARTCMD_CHECK_MODE_CRC)
    // CRC模式
    return crc16_verify(data, length);
    
#elif defined(UARTCMD_CHECK_MODE_CHECKSUM)
    // 校验和模式
    return verify_checksum(data, length);
    
#elif defined(UARTCMD_CHECK_MODE_NONE)
    // 无校验模式
    (void)data;
    (void)length;
    return 0; // 总是验证成功
    
#else
    // 默认使用CRC
    return crc16_verify(data, length);
#endif
}

// 计算数据的校验值（根据选择的校验模式）
static uint16_t calculate_check_value(const uint8_t *data, uint16_t length)
{
#if defined(UARTCMD_CHECK_MODE_CRC)
    // CRC模式
    return crc16_ccitt(data, length);
    
#elif defined(UARTCMD_CHECK_MODE_CHECKSUM)
    // 校验和模式
    return (uint16_t)calculate_checksum(data, length);
    
#elif defined(UARTCMD_CHECK_MODE_NONE)
    // 无校验模式
    (void)data;
    (void)length;
    return 0;
    
#else
    // 默认使用CRC
    return crc16_ccitt(data, length);
#endif
}

// 获取校验值的字节数
static uint8_t get_check_value_size(void)
{
#if defined(UARTCMD_CHECK_MODE_CRC)
    // CRC模式：2字节
    return 2;
    
#elif defined(UARTCMD_CHECK_MODE_CHECKSUM)
    // 校验和模式：1字节
    return 1;
    
#elif defined(UARTCMD_CHECK_MODE_NONE)
    // 无校验模式：0字节
    return 0;
    
#else
    // 默认使用CRC：2字节
    return 2;
#endif
}

// 缓冲区定义
uint8_t uartCmdBuffer_send[UARTCMD_BUFFER_SEND_SIZE];
uint8_t uartCmdBuffer_rcv[UARTCMD_BUFFER_RCV_SIZE];
uint16_t uartCmd_byte_counts = 0; // 收到的字节数

// 命令表
static const uart_cmd_t *cmd_table[16];
static uint8_t cmd_count = 0;

// 示例变量
static int val1 = 0;

// 不再需要单独的send_buffer，使用uartCmdBuffer_send

/**
 * @brief 初始化命令系统
 */
extern const uart_cmd_t user_cmds[];
extern uint16_t user_cmds_count;
void uart_cmd_init(void)
{
    cmd_count = 0;
    
    // 注册示例命令
    static const uart_cmd_t example_cmds[] = {
        {"test1", cmd_test1},
        {"set val1", cmd_set_val1},
        {"get val1", cmd_get_val1},
    };
    
    for (int i = 0; i < sizeof(example_cmds) / sizeof(example_cmds[0]); i++) {
        uart_cmd_register(&example_cmds[i]);
    }


    for (int i = 0; i < user_cmds_count; i++) {
        uart_cmd_register(&user_cmds[i]);
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
 * @brief 发送响应（类似printf功能）
 */
void uart_cmd_send_response(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    
    // 格式化字符串到uartCmdBuffer_send
    int len = vsnprintf((char *)uartCmdBuffer_send, UARTCMD_BUFFER_SEND_SIZE - 3, format, args);
    va_end(args);
    
    if (len > 0) {
        // 添加换行符
        if (len < UARTCMD_BUFFER_SEND_SIZE - 2) {
            uartCmdBuffer_send[len] = '\r';
            uartCmdBuffer_send[len + 1] = '\n';
            uartCmdBuffer_send[len + 2] = '\0';
            len += 2;
        }
        
        // 直接使用uartCmd_send发送数据
        uartCmd_send(uartCmdBuffer_send, len);
    }
}

/**
 * @brief 处理接收到的数据
 */
void uart_cmd_process(const uint8_t *data, uint16_t length)
{
    uint8_t check_size = get_check_value_size();
    
    if (length < check_size) {
        return; // 数据太短
    }
    
    // 验证数据
    if (verify_data(data, length) != 0) {
        uart_cmd_send_response("Data check failed");
        return;
    }
    
    // 提取命令数据（去掉校验字节）
    uint16_t cmd_length = length - check_size;
    
    
    // 直接在原数据上处理，不复制数据
    // 添加临时字符串结束符（临时修改缓冲区）
    uint8_t temp_char = data[cmd_length]; // 保存原始字符
    *((uint8_t *)(data + cmd_length)) = '\0'; // 临时添加结束符
    
    // 查找匹配的命令
    for (int i = 0; i < cmd_count; i++) {
        const uart_cmd_t *cmd = cmd_table[i];
        size_t cmd_len = strlen(cmd->cmd);
        
        // 检查命令是否匹配
        // 需要完全匹配：命令长度相等，或者命令后跟空格/结束符
        if (strncmp((const char *)data, cmd->cmd, cmd_len) == 0) {
            // 检查命令是否完全匹配
            if (cmd_length == cmd_len) {
                // 命令完全匹配，无参数
                
                // 恢复原始字符
                *((uint8_t *)(data + cmd_length)) = temp_char;
                
                // 执行命令处理函数
                cmd->handler(NULL);
                return;
            } else {
                // 检查命令后是否是空格或结束符
                char next_char = data[cmd_len];
                if (next_char == ' ' || next_char == '\0'|| next_char == '=') {
                    // 提取参数
                    const char *args = (const char *)(data + cmd_len);
                    // 跳过参数前的空格
                    while (*args == ' ') args++;
                    
                    // 检查参数是否为空（只有空格或结束符）
                    if (*args == '\0') {
                        // 参数为空，相当于无参数
                        
                        // 恢复原始字符
                        *((uint8_t *)(data + cmd_length)) = temp_char;
                        
                        // 执行命令处理函数
                        cmd->handler(NULL);
                        return;
                    }
                    
                    // 恢复原始字符
                    *((uint8_t *)(data + cmd_length)) = temp_char;
                    
                    // 执行命令处理函数
                    cmd->handler(args);
                    return;
                }
                // 如果不满足条件，继续查找下一个命令
            }
        }
    }
    
    // 恢复原始字符
    *((uint8_t *)(data + cmd_length)) = temp_char;
    
    // 没有找到匹配的命令
    // 调用未知命令处理函数
    unkonwnCmd();
}

// ==================== 示例命令处理函数 ====================

/**
 * @brief 测试命令1
 */
void cmd_test1(const char *args)
{
    (void)args; // 未使用参数
    uart_cmd_send_response("Test command1 executed successfully");
}


/**
 * @brief 设置val1的值
 */
void cmd_set_val1(const char *args)
{
    if (args == NULL || *args == '\0') {
        uart_cmd_send_response("Error: Need to specify value, e.g.: set val1=100");
        return;
    }
    
    int new_value;
    if (uart_cmd_extract_number(args, &new_value) == 0) {
        val1 = new_value;
        uart_cmd_send_response("Set successfully: val1 = %d", val1);
    } else {
        uart_cmd_send_response("Error: Cannot parse value");
    }
}

/**
 * @brief 获取val1的值
 */
void cmd_get_val1(const char *args)
{
    // get 命令不应该有参数
    if (args != NULL && *args != '\0') {
        // 检查参数是否包含 '='（错误语法，如 "get val1 = 100"）
        const char *ptr = args;
        while (*ptr != '\0') {
            if (*ptr == '=') {
                uart_cmd_send_response("Error: 'get val1' command does not take arguments with '='");
                return;
            }
            ptr++;
        }
        
        // 如果有其他参数（如 "get val1 s"），也返回错误
        uart_cmd_send_response("Error: 'get val1' command does not take arguments");
        return;
    }
    
    uart_cmd_send_response("val1 = %d", val1);
}

//发送完后该函数会被调用
void uart_send_finish_handle(void)
{

}
//接收完后该函数会被调用, 缓冲器数据会被丢弃
void uart_recive_finish_handle(void)
{
    //计算收到的字节数
    uartCmd_byte_counts = uartCmd_rcv_nums();
    
    // 如果有数据，处理命令
    if (uartCmd_byte_counts > 0) {
        //处理收到的数据
        uart_cmd_process(uartCmdBuffer_rcv, uartCmd_byte_counts);
    }
}

//未找到匹配指令
void unkonwnCmd()
{

}
