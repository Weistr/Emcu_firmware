#ifndef _BUZZER_H
#define _BUZZER_H
#include "main.h"
#define DO_h 0
#define RI_h 1
#define MI_h 2
#define FA_h 3
#define SO_h 4
#define LA_h 5 
#define SI_h 6
#define Void 7


//音阶频率
#define DO_h_frq 880
#define RI_h_frq 987
#define MI_h_frq 1108
#define FA_h_frq 1174
#define SO_h_frq 1318
#define LA_h_frq 1479
#define SI_h_frq 1661


extern uint8_t beeper_mode;

// 函数声明
extern uint8_t musicArr[50][2];
extern const uint8_t musicArr1[][2];
extern const uint8_t musicArr2[][2];

// 音乐播放控制函数
void buzzer_sonic_start(const uint8_t (*musicArray)[2]);
void buzzerTask_20ms(void);
void musicArr_deinit(void);

#endif // !_BUZZER_H
