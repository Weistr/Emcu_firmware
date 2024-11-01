#include "board.h"
#include "timer.h"
//实现功能：LED闪烁，PA0输出PWM


//PC13:LED
void gpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_GPIOC);  //GPIOA时钟使能
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); //设置PC13推挽输出
    /*Configure PA0 PA1 PA2(TIMER1 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
}

void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

void boardApp()
{
    static bool flag = 0;
    flag =! flag;
    if (flag)
    {
        gpio_bit_reset(GPIOC, GPIO_PIN_13);  //PC13置0
    }
    else
    {
        gpio_bit_set(GPIOC, GPIO_PIN_13);  //PC13置0
    }    
}

void boardInit(void)
{
    systick_config();
    gpioConfig();
    timerConfig();
}

