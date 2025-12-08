#include "ticksCost.h"
#include "main.h"
#include "gd32e23x.h"
#define getSysTickLOAD() (SysTick->LOAD) //SysTic计时器重载值
#define getSysTickVal() (SysTick->VAL) //SysTic计时器当前值
#define getSystemCoreClock() (SystemCoreClock) // 系统时钟

// 静态变量，用于记录计时状态
static volatile uint32_t startTickVal[TICKS_COST_MAX_TASKS] = {0};    // 开始时的SysTick值
static volatile uint32_t elapsedMs[TICKS_COST_MAX_TASKS] = {0};       // 经过的毫秒数
static volatile bool isMeasuring[TICKS_COST_MAX_TASKS] = {false};     // 是否正在测量

//该函数在sysTic中调用,系统最高优先级，1ms调用一次
void ticksCost_handle()
{
    for (uint8_t i = 0; i < TICKS_COST_MAX_TASKS; i++) {
        if (isMeasuring[i]) {
            elapsedMs[i]++;
        }
    }
}

//开始计时
void ticksCost_StartCount(uint8_t taskId)
{
    // 检查任务ID是否有效
    if (taskId >= TICKS_COST_MAX_TASKS) {
        return;
    }
    
    // 重置状态
    elapsedMs[taskId] = 0;
    isMeasuring[taskId] = true;
    
    // 记录当前的SysTick值
    startTickVal[taskId] = getSysTickVal();
}

//结束计时
//返回值：从调用ticksCost_StartCount，到调用ticksCost_StopCount之间花费多少微秒us
uint32_t ticksCost_StopCount(uint8_t taskId)
{
    // 检查任务ID是否有效
    if (taskId >= TICKS_COST_MAX_TASKS) {
        return 0;
    }
    
    // 如果任务没有在测量，返回0
    if (!isMeasuring[taskId]) {
        return 0;
    }
    
    uint32_t endTickVal;
    uint32_t totalTicks;
    uint32_t us;
    
    // 读取结束时的SysTick值
    endTickVal = getSysTickVal();
    
    // 停止测量
    isMeasuring[taskId] = false;
    
    // 获取该任务的开始值和经过的毫秒数
    uint32_t taskStartTickVal = startTickVal[taskId];
    uint32_t taskElapsedMs = elapsedMs[taskId];
    
    // 计算总tick数
    // 每毫秒有 (LOAD + 1) 个tick（从LOAD递减到0）
    // 总tick数 = 经过的毫秒数 * (LOAD + 1) + (开始值 - 结束值)
    // 注意：SysTick是递减计数器，所以开始值 >= 结束值（除非发生重载）
    
    // 获取LOAD值
    uint32_t loadVal = getSysTickLOAD();
    
    // 计算毫秒部分的tick数
    uint32_t msTicks = taskElapsedMs * (loadVal + 1);
    
    // 计算当前毫秒内的tick数
    // 如果endTickVal <= taskStartTickVal，说明没有发生重载
    // 如果endTickVal > taskStartTickVal，说明发生了重载，需要调整
    uint32_t currentMsTicks;
    if (endTickVal <= taskStartTickVal) {
        currentMsTicks = taskStartTickVal - endTickVal;
    } else {
        // 发生了重载，当前毫秒内的tick数 = (taskStartTickVal + 1) + (loadVal - endTickVal + 1)
        // 简化：loadVal - endTickVal + taskStartTickVal + 1
        currentMsTicks = loadVal - endTickVal + taskStartTickVal + 1;
    }
    
    // 总tick数
    totalTicks = msTicks + currentMsTicks;
    
    // 转换为微秒
    // us = (totalTicks * 1000000) / SystemCoreClock
    // 为了避免溢出，先除以SystemCoreClock再乘以1000000
    us = (uint32_t)(((uint64_t)totalTicks * 1000000ULL) / getSystemCoreClock());
    
    return us;
}
