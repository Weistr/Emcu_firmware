#include "main.h"
#include "task.h"
#include "gpio.h"
#include "systic.h"
#include "led.h"
#include "cmsis_delay.h"
#include "adc.h"
#include "dma.h"
#include "uartCmd.h"
#include "uart.h"
#include "timer.h"
#include "buzzer.h"
#include "ticksCost.h"
uint32_t debval[8];

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

// 用户命令处理函数实现
void cmd_userFun1(const char *args)
{

}
// 示例：解析数字
int userval1;
void cmd_set_userval1(const char *args)
{
}

void cmd_get_userval1(const char *args)
{

}



void basicTask()
{

    led_sta_update_20ms();
    buzzerTask_20ms();
    
}

TASK_COMPONENTS Task_Comps[]=
{
//状态  计数  周期  函数
	{0, 20, 20, basicTask},				/* task 1 Period： 2ms*/
//	{0, 500, 500, task_C},					/* task 6 Period： 500ms */
//	{0, 500, 500, task_D},					/* task 7 Period： 500ms */
//	{0, 500, 500, task_E},					/* task 8 Period： 500ms */

	/* Add new task here */
};
uint8_t Tasks_Max = sizeof(Task_Comps)/sizeof(Task_Comps[0]);


void main()
{
    systick_config();
    cmsis_blocked_delay_init();
    gpioConfig();
    adcConfig();
    dmaConfig();
    uartConfig();
    timerConfig();
    uart_cmd_init();
    while (1)
    {

        Task_Pro_Handler_Callback();
    }
}






