#include "board.h"

//实现功能：LED闪烁，PA0输出PWM


//PC13:LED
void gpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1);


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
        GPIO_BC(GPIOA) = GPIO_PIN_3;   //PC13置0
    }
    else
    {
        GPIO_BOP(GPIOA) = GPIO_PIN_3;  //PC13置0
    }    
    if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))GPIO_BOP(GPIOA) = GPIO_PIN_2;   //PC13置0
    else  GPIO_BC(GPIOA) = GPIO_PIN_2;  //PC13置0
    

}
void boardInit(void)
{
    systick_config();
    gpioConfig();

}

