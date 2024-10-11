#include "board.h"

//实现功能：LED闪烁，PA0输出PWM


//PC13:LED
void gpioConfig(void)
{
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
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

}

