#include "buzzer.h"
#include "gd32e23x_timer.h"
#include "gd32e23x_rcu.h"
#include <stdbool.h>
#include <string.h>

// 定义多个音乐数组（常量）
const uint8_t musicArr1[][2] = 
{
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {0,0}
};

const uint8_t musicArr2[][2] = 
{
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {0,0}
};

// 用户可修改的音乐数组（初始化为默认音乐）
uint8_t musicArr[50][2] = 
{
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {DO_h,8},
    {RI_h,8},
    {MI_h,8},
    {Void,40},
    {0, 0} // 结束标记
};

// 全局变量用于跟踪播放状态
static bool isPlaying = false;
static uint16_t currentNoteIndex = 0;
static uint16_t currentNoteDuration = 0;
static uint16_t elapsedTime = 0; // 单位：20ms
static uint16_t totalNotes = 0; // 当前音乐数组中的音符数量
static const uint8_t (*currentMusicArr)[2] = musicArr1;
static uint16_t currentMusicSize = sizeof(musicArr1) / sizeof(musicArr1[0]);

// 音符频率数组
static const uint16_t noteFrequencies[] = {
    DO_h_frq,  // 0: DO_h
    RI_h_frq,  // 1: RI_h
    MI_h_frq,  // 2: MI_h
    FA_h_frq,  // 3: FA_h
    SO_h_frq,  // 4: SO_h
    LA_h_frq,  // 5: LA_h
    SI_h_frq,  // 6: SI_h
    0          // 7: Void (静音)
};

// 设置PWM频率
static void setPwmFrequency(uint16_t frequency)
{
    if (frequency == 0) {
        // 静音：禁用通道输出
        timer_channel_output_state_config(TIMER2, TIMER_CH_1, TIMER_CCX_DISABLE);
        return;
    }
    
    // 启用通道输出
    timer_channel_output_state_config(TIMER2, TIMER_CH_1, TIMER_CCX_ENABLE);
    
    // 计算周期值
    // 定时器时钟频率 = 72MHz / (71+1) = 1MHz
    // 周期值 = 时钟频率 / PWM频率 - 1
    uint32_t timerClock = 1000000; // 1MHz
    uint32_t period = timerClock / frequency;
    
    // 确保period至少为2（因为需要50%占空比）
    if (period < 2) {
        period = 2;
    }
    
    // 设置自动重装载值（周期）
    timer_autoreload_value_config(TIMER2, period - 1);
    
    // 设置占空比为50%
    uint32_t pulse = period / 2;
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, pulse);
}

// 自动计算音乐数组中的音符数量
static uint16_t calculate_music_size(const uint8_t (*musicArray)[2], uint16_t maxSize)
{
    // 查找第一个全零的元素作为结束标记
    for (uint16_t i = 0; i < maxSize; i++) {
        if (musicArray[i][0] == 0 && musicArray[i][1] == 0) {
            return i;
        }
    }
    return maxSize;
}

// 清空用户可修改的音乐数组
void musicArr_deinit(void)
{
    memset(musicArr, 0, sizeof(musicArr));
}

// 开始播放音乐
// 参数：musicArray - 音乐数组指针，可以为：
//      NULL: 使用当前设置的音乐数组
//      musicArr1, musicArr2: 预设音乐数组
//      musicArr: 用户自定义音乐数组
void buzzer_sonic_start(const uint8_t (*musicArray)[2])
{
    // 设置音乐数组
    if (musicArray != NULL) {
            // 自动计算大小
            currentMusicArr = musicArray;
        }
    
    
    // 重置播放状态
    isPlaying = true;
    currentNoteIndex = 0;
    currentNoteDuration = 0;
    elapsedTime = 0;
    
    // 自动计算音符数量
    totalNotes = calculate_music_size(currentMusicArr, currentMusicSize);
    
    // 开始播放第一个音符
    if (totalNotes > 0) {
        uint8_t note = currentMusicArr[0][0];
        if (note <= 7) {
            setPwmFrequency(noteFrequencies[note]);
        }
        currentNoteDuration = currentMusicArr[0][1];
    }
}

void buzzer_sonic(void)
{
    if (!isPlaying) {
        return;
    }
    
    elapsedTime++;
    
    // 检查当前音符是否播放完毕
    if (elapsedTime >= currentNoteDuration) {
        // 移动到下一个音符
        currentNoteIndex++;
        elapsedTime = 0;
        
        // 检查是否所有音符都播放完毕
        if (currentNoteIndex >= totalNotes) {
            isPlaying = false;
            // 停止发声
            setPwmFrequency(0);
            return;
        }
        
        // 播放下一个音符
        uint8_t note = currentMusicArr[currentNoteIndex][0];
        if (note <= 7) {
            setPwmFrequency(noteFrequencies[note]);
        }
        currentNoteDuration = currentMusicArr[currentNoteIndex][1];
    }
}

void buzzerTask_20ms(void)
{
    buzzer_sonic();
}
