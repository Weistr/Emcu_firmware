#ifndef _TICKSCOST_H
#define _TICKSCOST_H

#include <stdint.h>
#include <stdbool.h>

// 最大任务数
#define TICKS_COST_MAX_TASKS 3

// 开始计时
// taskId: 任务ID，范围0到(TICKS_COST_MAX_TASKS-1)
void ticksCost_StartCount(uint8_t taskId);

// 结束计时
// taskId: 任务ID，范围0到(TICKS_COST_MAX_TASKS-1)
// 返回值：从调用ticksCost_StartCount，到调用ticksCost_StopCount之间花费多少微秒us
uint32_t ticksCost_StopCount(uint8_t taskId);

// SysTick中断处理函数，每1ms调用一次
void ticksCost_handle(void);

#endif // !_TICKSCOST_H
