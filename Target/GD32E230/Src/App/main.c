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

//5ms定时器
void timer5ms()
{
    
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);//启动ADC转换
    ticksCost_StartCount(0);
}
//adc扫描完成后调用
void adcScanHandle()
{
    debval[0] = ticksCost_StopCount(0);
}



void basicTask()
{

    led_sta_update_20ms();
    buzzerTask_20ms();
    
}
extern void ECB_task_100ms(void);
TASK_COMPONENTS Task_Comps[]=
{
//状态  计数  周期  函数
	{0, 20, 20, basicTask},				/* task 1 Period： 2ms*/
	{0, 100, 100, ECB_task_100ms},					/* task 5 Period： 500ms */
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






