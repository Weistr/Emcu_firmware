#include "uartCmd.h"

// 用户命令处理函数声明
void cmd_ble_linked(const char *args);
void cmd_ble_unlinked(const char *args);
void cmd_get_status(const char *args);


// 用户命令数组
const uart_cmd_t user_cmds[] = {
    {"AT+LINK\r\n+LINK:Online\r\nOK\r\n", cmd_ble_linked},
    {"CONNECT OK\r\n", cmd_ble_linked},
    {"AT+LINK\r\n+LINK:Offline\r\nOK\r\n", cmd_ble_unlinked},
    {"DISCONNECT\r\n", cmd_ble_unlinked},
    {"get status",cmd_get_status}


};

// 用户命令数组大小
uint16_t user_cmds_count = sizeof(user_cmds) / sizeof(user_cmds[0]);



//蓝牙模块
uint8_t flag_ble_link = 0xFF;
void ECB_task_100ms()
{
    static uint16_t cnt1 = 0;
    if(cnt1 < 5)cnt1++;//每500ms执行1次，查询蓝牙状态
    else if(flag_ble_link==0xFF)
    {
        cnt1 = 0;
        uart_cmd_send_response("AT+LINK?");
    }

}



//蓝牙已连接
void cmd_ble_linked(const char *args)
{
    (void)args; // 未使用参数

    flag_ble_link = 1;
}

//蓝牙未连接
void cmd_ble_unlinked(const char *args)
{
    (void)args; // 未使用参数

    flag_ble_link = 0;
}

void cmd_get_status(const char *args)
{

}