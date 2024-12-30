//###########################################################################
//
// FILE:   F2806x_EPwm.c
//
// TITLE:  F2806x EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File

//
// InitEPwm -  This function initializes the EPwm(s) to a known state.
//
void
InitEPwm(void)
{

    //////////////////////////////////////////////////////////////////////////////////
    // Time-Base Control Register (TBCTL)
    ////////////////////////////////////////////////////////////////////////////////////


    //Emulation Mode Bits. 仿真时可能用到，
    //00: Stop after the next time-base counter increment or decrement
    //01:Stop when counter completes a whole cycle:
    //1x:Free run
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;//Free run

    //Phase Direction Bit.R/W-0 主要用于读取当前计数器状态
    //This bit is only used when the time-base counter is configured in the up-down-count mode.
    //In the up-count and down-count modes this bit is ignored.
    //0:Count down after the synchronization event.
    //1:Count up after the synchronization event.
    EPwm1Regs.TBCTL.bit.PHSDIR == 1;


    //Time-base Clock Prescale Bits //分频设置
    //These bits determine part of the time-base clock prescale value.TBCLK = SYSCLKOUT / (HSPCLKDIV × CLKDIV)
    //In the up-count and down-count modes this bit is ignored.
    //=0:/1(defult); =1:/2; =2:/4; 3:/8; 7:/128
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;


    //High Speed Time-base Clock Prescale Bits//分频设置
    //This divisor emulates the HSPCLK in the TMS320x281x system as used on the Event Manager (EV) peripheral.
    //EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;


    //Software Forced Synchronization Pulse//使用软件强制同步
    //0：no effect
    //1:Writing a 1 forces a one-time synchronization pulse to be generated.
    EPwm1Regs.TBCTL.bit.SWFSYNC = 0;


    //Synchronization Output Select. These bits select the source of the EPWMxSYNCO signal.选择同步信号来源
    //00：EPWMxSYNC
    //01:CTR = zero: Time-base counter equal to zero (TBCTR = 0x0000)
    //10:CTR = CMPB : Time-base counter equal to counter-compare B (TBCTR = CMPB)
    //11:Disable EPWMxSYNCO signal
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 3;


    //Active Period Register Load From Shadow Register Select 使能TBPRD是否从影子寄存器装载，还是直接控制
    //0:The period register (TBPRD) is loaded from its shadow register when the time-base counter, TBCTR, is equal to zero.
    //1:Load the TBPRD register immediately without using a shadow register.
    EPwm1Regs.TBCTL.bit.PRDLD = 0;


    //Counter Register Load From Phase Register Enable 是否使能从TBPHS装载到TBCTR
    //0:Do not load the time-base counter (TBCTR) from the time-base phase register (TBPHS)
    //1:do load
    EPwm1Regs.TBCTL.bit.PHSEN = 0;


    //Counter Mode
    //00:Up-count mode
    //01:Down-count mode
    //10 Up-down-count mode
    //11 Stop-freeze counter operation (default on reset)
    EPwm1Regs.TBCTL.bit.PHSEN = 0;


    //////////////////////////////////////////////////////////////////////////////////
    // Time-Base Status Register (TBSTS)
    ////////////////////////////////////////////////////////////////////////////////////

    //Time-Base Counter Max Latched Status Bit 计时器是否达到过最大值，写1清除标志位
    //0 Reading a 0 indicates the time-base counter never reached its maximum value. Writing a 0 will  have no effect.
    //1 Reading a 1 on this bit indicates that the tim EPwm1Regs.TBCTL.bit.PHSEN = 0;
    if(EPwm1Regs.TBSTS.bit.CTRMAX == 1);


    //Input Synchronization Latched Status Bit 外部同步信号是否发生过，写1清除标志位
    //0 Writing a 0 will have no effect. Reading a 0 indicates no external synchronization event has  occurred.
    //1 Reading a 1 on this bit indicates that an external synchronization event has occurred (EPWMxSYNCI). Writing a 1 to this bit will clear the latched event.    EPwm1Regs.TBSTS.bit.CTRMAX == 1;
    if(EPwm1Regs.TBSTS.bit.SYNCI == 1);

    //当前计数器状态，向上计数/向下计数
    //Time-Base Counter Direction Status Bit. At reset, the counter is frozen; therefore,
    //this bit has no meaning. To make this bit meaningful, you must first set the appropriate mode via TBCTL[CTRMODE].
    //0 Time-Base Counter is currently counting down.
    //1 Time-Base Counter is currently counting up
    if(EPwm1Regs.TBSTS.bit.CTRDIR == 1);


    //////////////////////////////////////////////////////////////////////////////////
    // Time-Base Phase Register (TBPHS)相位寄存器（0~FFFF）见功能模块图
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.TBPHS.all = 0;//

    //////////////////////////////////////////////////////////////////////////////////
    // Time-Base Counter Register (TBCTR) 0~FFFF 计数器 见功能模块图
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.TBCTR = 0;//计数器清零


    //////////////////////////////////////////////////////////////////////////////////
    // Time-Base Period Register 0~FFFF 周期计数器 见功能模块图
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.TBPRD = 1000;//

















    //////////////////////////////////////////////////////////////////////////////////
    // Counter-Compare Submodule Registers 计数器比较器，与PWM占空比设置有关
    ////////////////////////////////////////////////////////////////////////////////////


    //比较器B影子寄存器满标志位
    //SHDWBFULL Counter-compare B (CMPB) Shadow Register Full Status Flag
    //This bit self clears once a load-strobe occurs.
    //0 CMPB shadow FIFO not full yet
    //1 Indicates the CMPB shadow FIFO is full; a CPU write will overwrite current shadow value.
    if(EPwm1Regs.CMPCTL.bit.SHDWBFULL == 1);
    //比较器A影子寄存器满标志位
    if(EPwm1Regs.CMPCTL.bit.SHDWAFULL == 1);


    //CMPB模式，是否使能影子寄存器
    //Counter-compare B (CMPB) Register Operating Mode
    //0 Shadow mode. Operates as a double buffer. All writes via the CPU access the shadow register.
    //1 Immediate mode. Only the active compare B register is used. All writes and reads directly access 
    //the active register for immediate compare action.   
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;
    //CMPA模式，是否使能影子寄存器  
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;


    //CMPB模式，设置何时装载影子寄存器数值
    //Active Counter-Compare B (CMPB) Load From Shadow Select Mode
    //This bit has no effect in immediate mode (CMPCTL[SHDWBMODE] = 1).
    //00 Load on CTR = Zero: Time-base counter equal to zero (TBCTR = 0x0000)
    //01 Load on CTR = PRD: Time-base counter equal to period (TBCTR = TBPRD)
    //10 Load on either CTR = Zero or CTR = PRD
    //11 Freeze (no loads possible) 
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 0;
    //CMPA模式，是否使能影子寄存器  
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;

    //////////////////////////////////////////////////////////////////////////////////
    //CMPB寄存器，向它写默认是向影子寄存器写
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.CMPB = 0;

    //////////////////////////////////////////////////////////////////////////////////
    //CMPA寄存器，向它写默认是向影子寄存器写
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.CMPA.all = 0;










    //////////////////////////////////////////////////////////////////////////////////
    // AQ模块
    ////////////////////////////////////////////////////////////////////////////////////
    // Action-Qualifier Output A Control Register (AQCTLA)动作设置寄存器A
    ////////////////////////////////////////////////////////////////////////////////////
    
    //当向下计数时CTR=CMPB时动作
    //Action when the time-base counter equals the active CMPB register and the counter is decrementing.
    //00 Do nothing (action disabled)
    //01 Clear: force EPWMxA output low.
    //10 Set: force EPWMxA output high.
    //11 Toggle EPWMxA output: low output signal will be forced high, and a high signal will be forced low.
    EPwm1Regs.AQCTLA.bit.CBD = 0;
    //当向上计数时CTR=CMPB时动作    
    //Action when the counter equals the active CMPB register and the counter is incrementing.
    EPwm1Regs.AQCTLA.bit.CBU = 0;
    //当向下计数时CTR=CMPA时动作
    EPwm1Regs.AQCTLA.bit.CAD = 0;
    //当向下计数时CTR=CMPB时动作
    EPwm1Regs.AQCTLA.bit.CAU = 0;
    //CTR=PRD时动作
    //Note: By definition, in count up-down mode when the counter equals period the direction is defined as 0 or 
    //counting down.
    EPwm1Regs.AQCTLA.bit.PRD = 0;
    //CTR=ZRO时动作
    //Note: By definition, in count up-down mode when the counter equals period the direction is defined as 1 or 
    //counting down.
    EPwm1Regs.AQCTLA.bit.ZRO = 0;    

    //////////////////////////////////////////////////////////////////////////////////
    // Action-Qualifier Output B Control Register (AQCTLB)动作设置寄存器B,同A
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.AQCTLB.bit.CBD = 0;
    EPwm1Regs.AQCTLB.bit.CBU = 0;
    EPwm1Regs.AQCTLB.bit.CAD = 0;
    EPwm1Regs.AQCTLB.bit.CAU = 0;
    EPwm1Regs.AQCTLB.bit.PRD = 0;
    EPwm1Regs.AQCTLB.bit.ZRO = 0;  

    //////////////////////////////////////////////////////////////////////////////////
    // Action-Qualifier Software Force Register (AQSFRC)
    ////////////////////////////////////////////////////////////////////////////////////
    //设置AQCSFRC影子寄存器何时装载
    //AQCSFRC Active Register Reload From Shadow Options
    //00 Load on event counter equals zero
    //01 Load on event counter equals period
    //10 Load on event counter equals zero or counter equals period
    //11 Load immediately (the active register is directly accessed by the CPU and is not loaded from the 
    //shadow register)
    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;

    //软件单次触发
    //One-Time Software Forced Event on Output B
    //0 Writing a 0 (zero) has no effect. Always reads back a 0
    //This bit is auto cleared once a write to this register is complete (that is, a forced event is initiated).
    //This is a one-shot forced event. It can be overridden by another subsequent event on output B.
    //1 Initiates a single s/w forced event
    EPwm1Regs.AQSFRC.bit.OTSFB = 0;

    //  
    //Action when One-Time Software Force B Is invoked
    //00 Does nothing (action disabled)
    //01 Clear (low)
    //10 Set (high)
    //11 Toggle (Low -> High, High -> Low)
    EPwm1Regs.AQSFRC.bit.ACTSFB = 0; 

    //同OTSFB一样
    EPwm1Regs.AQSFRC.bit.ACTSFA = 0;
    EPwm1Regs.AQSFRC.bit.OTSFA = 0;


    //////////////////////////////////////////////////////////////////////////////////
    // 4 Action-Qualifier Continuous Software Force Register (AQCSFRC)
    ////////////////////////////////////////////////////////////////////////////////////
    //
    //Continuous Software Force on Output B
    //In immediate mode, a continuous force takes effect on the next TBCLK edge.
    //In shadow mode, a continuous force takes effect on the next TBCLK edge after a shadow load into the 
    //active register. To configure shadow mode, use AQSFRC[RLDCSF].
    //00 Software forcing is disabled and has no effect
    //01 Forces a continuous low on output B
    //10 Forces a continuous high on output B
    //11 Software forcing is disabled and has no effect
    EPwm1Regs.AQCSFRC.bit.CSFB = 0;     

    //Continuous Software Force on Output A
    //In immediate mode, a continuous force takes effect on the next TBCLK edge.
    //In shadow mode, a continuous force takes effect on the next TBCLK edge after a shadow load into the 
    //active register.
    //00 Software forcing is disabled and has no effect
    //01 Forces a continuous low on output A
    //10 Forces a continuous high on output A
    //11 Software forcing is disabled and has no effec   
    EPwm1Regs.AQCSFRC.bit.CSFA = 0;  














    //////////////////////////////////////////////////////////////////////////////////
    // Dead-Band Generator Control (DBCTL) Register 死区控制
    ////////////////////////////////////////////////////////////////////////////////////
    //DB计数器二分频
    //Half Cycle Clocking Enable Bit:
    //0 Full cycle clocking enabled. The dead-band counters are clocked at the TBCLK rate.
    //1 Half cycle clocking enabled. The dead-band counters are clocked at TBCLK*2.
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 0;

    //详情见BLOCK图，bit5 控制S5 bit4控制S4
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;

    //详情见BLOCK图，bit3控制S3 bit2控制S2
    EPwm1Regs.DBCTL.bit.POLSEL = 0;    

    //详情见BLOCK图，bit1控制S1 bit0控制S0
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0; 

    //////////////////////////////////////////////////////////////////////////////////
    // Dead-Band Generator Rising Edge Delay (DBRED) Register 死区时间设置(10bit)
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.DBRED = 0;
    //////////////////////////////////////////////////////////////////////////////////
    // Dead-Band Generator Falling Edge Delay (DBFED) Register 死区时间设置(10bit)
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.DBFED = 0;












    //////////////////////////////////////////////////////////////////////////////////
    // PWM-Chopper Control (PCCTL) Register PWM chopper模式
    ////////////////////////////////////////////////////////////////////////////////////
    //Chopping Clock Duty Cycle
    //000 Duty = 1/8 (12.5%)
    //001 Duty = 2/8 (25.0%)
    //010 Duty = 3/8 (37.5%)
    //011 Duty = 4/8 (50.0%)
    //100 Duty = 5/8 (62.5%)
    //101 Duty = 6/8 (75.0%)
    //110 Duty = 7/8 (87.5%)
    EPwm1Regs.PCCTL.bit.CHPDUTY = 0; //

    //Chopping Clock Frequency
    //000 Divide by 1 (no prescale, = 11.25 MHz at 90 MHz SYSCLKOUT)
    //001 Divide by 2 (5.63 MHz at 90 MHz SYSCLKOUT)
    //010 Divide by 3 (3.75 MHz at 90 MHz SYSCLKOUT)
    //011 Divide by 4 (2.81 MHz at 90 MHz SYSCLKOUT)
    //100 Divide by 5 (2.25 MHz at 90 MHz SYSCLKOUT)
    //101 Divide by 6 (1.88 MHz at 90 MHz SYSCLKOUT)
    //110 Divide by 7 (1.61 MHz at 90 MHz SYSCLKOUT)
    //111 Divide by 8 (1.41 MHz at 90 MHz SYSCLKOUT)
    EPwm1Regs.PCCTL.bit.CHPFREQ = 0;

    //0000 1 x SYSCLKOUT / 8 wide ( = 88.89 nS at 90 MHz SYSCLKOUT)
    //0001 2 x SYSCLKOUT / 8 wide ( = 177.8 nS at 90 MHz SYSCLKOUT)
    //0010 3 x SYSCLKOUT / 8 wide ( = 266.7 nS at 90 MHz SYSCLKOUT)
    //0011 4 x SYSCLKOUT / 8 wide ( = 355.6 nS at 90 MHz SYSCLKOUT)
    //同上递增
    EPwm1Regs.PCCTL.bit.OSHTWTH = 0;

    //使能/失能该功能
    EPwm1Regs.PCCTL.bit.CHPEN = 0;//disable
















    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Select Register (TZSEL) 故障处理,类似PWM刹车模块，该寄存器用于选择使能用于触发故障的通道
    ////////////////////////////////////////////////////////////////////////////////////
    //使能失能比较器通道作为oneShot故障输入
    //Digital Compare Output B Event 1 Select
    //0 Disable DCBEVT1 as one-shot-trip source for this ePWM module.
    //1 Enable DCBEVT1 as one-shot-trip source for this ePWM module
    EPwm1Regs.TZSEL.bit.DCBEVT1 = 0;//disable 比较器B
    EPwm1Regs.TZSEL.bit.DCAEVT1 = 0;//disable 比较器A

    //使能失能TZx通道作为oneShot故障输入
    EPwm1Regs.TZSEL.bit.OSHT6 = 0;
    EPwm1Regs.TZSEL.bit.OSHT5 = 0;
    EPwm1Regs.TZSEL.bit.OSHT4 = 0;
    EPwm1Regs.TZSEL.bit.OSHT3 = 0;
    EPwm1Regs.TZSEL.bit.OSHT2 = 0;
    EPwm1Regs.TZSEL.bit.OSHT1 = 0;

    //使能失能比较器通道作为CBC故障输入
    //Digital Compare Output B Event 2 Select
    //0 Disable DCBEVT2 as a CBC trip source for this ePWM module
    //1 Enable DCBEVT2 as a CBC trip source for this ePWM module
    EPwm1Regs.TZSEL.bit.DCBEVT2 = 0;//disable 比较器B
    EPwm1Regs.TZSEL.bit.DCAEVT2 = 0;//disable 比较器A

    //使能失能TZx通道作为CBC故障输入
    EPwm1Regs.TZSEL.bit.CBC1 = 0;
    EPwm1Regs.TZSEL.bit.CBC2 = 0;
    EPwm1Regs.TZSEL.bit.CBC3 = 0;
    EPwm1Regs.TZSEL.bit.CBC4 = 0;
    EPwm1Regs.TZSEL.bit.CBC5 = 0;
    EPwm1Regs.TZSEL.bit.CBC6 = 0;


    //////////////////////////////////////////////////////////////////////////////////
    // Trip Zone Digital Compare Event Select Register (TZDCSEL)比较器事件触发设置
    ////////////////////////////////////////////////////////////////////////////////////

    //Digital Compare Output B Event 2 Selection
    // 000 Event disabled
    // 001 DCBH = low, DCBL = don't care
    // 010 DCBH = high, DCBL = don't care
    // 011 DCBL = low, DCBH = don't care
    // 100 DCBL = high, DCBH = don't care
    // 101 DCBL = high, DCBH = low
    EPwm1Regs.TZDCSEL.bit.DCBEVT2 = 0;
    //Digital Compare Output B Event 1 Selection  DCBEVT1同DCBEVT2
    EPwm1Regs.TZDCSEL.bit.DCBEVT1 = 0;
    //Digital Compare Output A Event 2 Selection
    EPwm1Regs.TZDCSEL.bit.DCAEVT2 = 0;
    //Digital Compare Output A Event 1 Selection
    EPwm1Regs.TZDCSEL.bit.DCAEVT1 = 0;    

    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Control Register (TZCTL)TZ控制寄存器，设置故障事件发生要做什么动作
    ////////////////////////////////////////////////////////////////////////////////////
    // Digital Compare Output B Event 2 Action On EPWMxB:
    // 00 High-impedance (EPWMxB = High-impedance state)
    // 01 Force EPWMxB to a high state.
    // 10 Force EPWMxB to a low state.
    // 11 Do Nothing, trip action is disabled
    EPwm1Regs.TZCTL.bit.DCBEVT2 = 0;
    //Digital Compare Output B Event 1 Action On EPWMxB:
    EPwm1Regs.TZCTL.bit.DCBEVT1 = 0;
    //Digital Compare Output A Event 2 Action On EPWMxA:
    EPwm1Regs.TZCTL.bit.DCAEVT2 = 0;
    //Digital Compare Output A Event 2 Action On EPWMxA:
    EPwm1Regs.TZCTL.bit.DCAEVT1 = 0;    
    //When a trip event occurs the following action is taken on output EPWMxB. Which trip-zone pins 
    //can cause an event is defined in the TZSEL register.
    EPwm1Regs.TZCTL.bit.TZB = 0; 
    //When a trip event occurs the following action is taken on output EPWMxA. Which trip-zone pins 
    //can cause an event is defined in the TZSEL register.
    EPwm1Regs.TZCTL.bit.TZA = 0;       

    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Enable Interrupt Register (TZEINT)TZ中断使能
    ////////////////////////////////////////////////////////////////////////////////////
    // Digital Comparator Output B Event 2 Interrupt Enable
    // 0 Disabled
    // 1 Enabled
    EPwm1Regs.TZEINT.bit.DCBEVT2 = 0;
    //Digital Comparator Output B Event 1 Interrupt Enable
    EPwm1Regs.TZEINT.bit.DCBEVT1 = 0;
    //Digital Comparator Output A Event 2 Interrupt Enable
    EPwm1Regs.TZEINT.bit.DCAEVT2 = 0;
    //Digital Comparator Output A Event 1 Interrupt Enable
    EPwm1Regs.TZEINT.bit.DCAEVT1 = 0;   
    //Digital Comparator Output A Event 2 Interrupt Enable
    EPwm1Regs.TZEINT.bit.DCAEVT2 = 0; 
    // Trip-zone One-Shot Interrupt Enable
    // 0 Disable one-shot interrupt generation
    // 1 Enable Interrupt generation; a one-shot trip event will cause a EPWMx_TZINT PIE interrupt.
    EPwm1Regs.TZEINT.bit.OST = 0; 
    //Trip-zone Cycle-by-Cycle Interrupt Enable
    EPwm1Regs.TZEINT.bit.CBC = 0; 

    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Flag Register (TZFLG)(R-0), 故障标志位读取
    ////////////////////////////////////////////////////////////////////////////////////
    // Latched Status Flag for Digital Compare Output B Event 2
    // 0 Indicates no trip event has occurred on DCBEVT2
    // 1 Indicates a trip event has occurred for the event defined for DCBEVT2
    if(EPwm1Regs.TZFLG.bit.DCBEVT2 == 0);
    //Latched Status Flag for Digital Compare Output B Event 1
    if(EPwm1Regs.TZFLG.bit.DCBEVT1 == 0);
    //Latched Status Flag for Digital Compare Output A Event 2
    if(EPwm1Regs.TZFLG.bit.DCAEVT2 == 0);
    //Latched Status Flag for Digital Compare Output A Event 1
    if(EPwm1Regs.TZFLG.bit.DCAEVT1 == 0);    
    // Latched Status Flag for A One-Shot Trip Event
    // 0 No one-shot trip event has occurred.
    // 1 Indicates a trip event has occurred on a pin selected as a one-shot trip source.
    // This bit is cleared by writing the appropriate value to the TZCLR register .
    if(EPwm1Regs.TZFLG.bit.OST == 0);
    // Latched Status Flag for Cycle-By-Cycle Trip Event
    // 0 No cycle-by-cycle trip event has occurred.
    // 1 Indicates a trip event has occurred on a signal selected as a cycle-by-cycle trip source. The 
    // TZFLG[CBC] bit will remain set until it is manually cleared by the user. If the cycle-by-cycle trip 
    // event is still present when the CBC bit is cleared, then CBC will be immediately set again. The 
    // specified condition on the signal is automatically cleared when the ePWM time-base counter 
    // reaches zero (TBCTR = 0x0000) if the trip condition is no longer present. The condition on the 
    // signal is only cleared when the TBCTR = 0x0000 no matter where in the cycle the CBC flag is 
    // cleared.
    // This bit is cleared by writing the appropriate value to the TZCLR register 
    if(EPwm1Regs.TZFLG.bit.CBC == 0);   
    // Latched Trip Interrupt Status Flag
    // 0 Indicates no interrupt has been generated.
    // 1 Indicates an EPWMx_TZINT PIE interrupt was generated because of a trip condition.
    // No further EPWMx_TZINT PIE interrupts will be generated until this flag is cleared. If the interrupt 
    // flag is cleared when either CBC or OST is set, then another interrupt pulse will be generated. 
    // Clearing all flag bits will prevent further interrupts.
    // This bit is cleared by writing the appropriate value to the TZCLR register .    
    if(EPwm1Regs.TZFLG.bit.INT == 0); 

    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Clear Register (TZCLR) TZ标志位清除（写1）含义见上一个寄存器
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.TZCLR.bit.DCBEVT2 = 0;
    EPwm1Regs.TZCLR.bit.DCBEVT1 = 0;
    EPwm1Regs.TZCLR.bit.DCAEVT2 = 0;
    EPwm1Regs.TZCLR.bit.DCAEVT1 = 0;
    EPwm1Regs.TZCLR.bit.OST = 0;
    EPwm1Regs.TZCLR.bit.CBC = 0;
    EPwm1Regs.TZCLR.bit.INT = 0;
    //////////////////////////////////////////////////////////////////////////////////
    // Trip-Zone Force Register (TZFRC) TZ事件强制置位（写1）
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.TZFRC.bit.DCBEVT2 = 0;
    EPwm1Regs.TZFRC.bit.DCBEVT1 = 0;
    EPwm1Regs.TZFRC.bit.DCAEVT2 = 0;
    EPwm1Regs.TZFRC.bit.DCAEVT1 = 0;
    EPwm1Regs.TZFRC.bit.OST = 0;
    EPwm1Regs.TZFRC.bit.CBC = 0;














    //////////////////////////////////////////////////////////////////////////////////
    // Event-Trigger Selection Register (ETSEL)事件触发选择，用于PWM触发ADC或者中断
    ////////////////////////////////////////////////////////////////////////////////////
    //使能SOCB
    // Enable the ADC Start of Conversion B (EPWMxSOCB) Pulse
    // 0 Disable EPWMxSOCB.
    // 1 Enable EPWMxSOCB pulse.
    EPwm1Regs.ETSEL.bit.SOCBEN = 0;

    //选择SOCB使用哪个事件
    // EPWMxSOCB Selection Options
    // These bits determine when a EPWMxSOCB pulse will be generated.
    // 000 Enable DCBEVT1.soc event
    // 001 Enable event time-base counter equal to zero. (TBCTR = 0x0000)
    // 010 Enable event time-base counter equal to period (TBCTR = TBPRD)
    // 011 Enable event time-base counter equal to zero or period (TBCTR = 0x0000 or TBCTR = TBPRD). 
    // This mode is useful in up-down count mode.
    // 100 Enable event time-base counter equal to CMPA when the timer is incrementing.
    // 101 Enable event time-base counter equal to CMPA when the timer is decrementing.
    // 110 Enable event: time-base counter equal to CMPB when the timer is incrementing.
    // 111 Enable event: time-base counter equal to CMPB when the timer is decrementing
    EPwm1Regs.ETSEL.bit.SOCBSEL = 0;

    //同B
    //Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;
    //EPWMxSOCA Selection Options
    EPwm1Regs.ETSEL.bit.SOCASEL = 0;

    //EPWM中断设置
    // Enable ePWM Interrupt (EPWMx_INT) Generation
    // 0 Disable EPWMx_INT generation
    // 1 Enable EPWMx_INT generation
    EPwm1Regs.ETSEL.bit.INTEN = 0;
    //ePWM Interrupt (EPWMx_INT) Selection Options
    //EPWM中断事件选择
    // 000 Reserved
    // 001 Enable event time-base counter equal to zero. (TBCTR = 0x0000)
    // 010 Enable event time-base counter equal to period (TBCTR = TBPRD)
    // 011 Enable event time-base counter equal to zero or period (TBCTR = 0x0000 or TBCTR = TBPRD). 
    // This mode is useful in up-down count mode.
    // 100 Enable event time-base counter equal to CMPA when the timer is incrementing.
    // 101 Enable event time-base counter equal to CMPA when the timer is decrementing.
    // 110 Enable event: time-base counter equal to CMPB when the timer is incrementing.
    // 111 Enable event: time-base counter equal to CMPB when the timer is decrementing.
    EPwm1Regs.ETSEL.bit.INTSEL = 0;


    //////////////////////////////////////////////////////////////////////////////////
    // Event-Trigger Prescale Register (ETPS)
    ////////////////////////////////////////////////////////////////////////////////////
    //读取有几个SOCB触发事件发生
    // ePWM ADC Start-of-Conversion B Event (EPWMxSOCB) Counter Register
    // These bits indicate how many selected ETSEL[SOCBSEL] events have occurred:
    // 00 No events have occurred.
    // 01 1 event has occurred.
    // 10 2 events have occurred.
    // 11 3 events have occurred.
    EPwm1Regs.ETPS.bit.SOCBCNT = 0;

    //在第几个事件触发时输出SOCB
    // ePWM ADC Start-of-Conversion B Event (EPWMxSOCB) Period Select
    // These bits determine how many selected ETSEL[SOCBSEL] events need to occur before an 
    // EPWMxSOCB pulse is generated. To be generated, the pulse must be enabled (ETSEL[SOCBEN] 
    // = 1). The SOCB pulse will be generated even if the status flag is set from a previous start of 
    // conversion (ETFLG[SOCB] = 1). Once the SOCB pulse is generated, the ETPS[SOCBCNT] bits 
    // will automatically be cleared.
    // 00 Disable the SOCB event counter. No EPWMxSOCB pulse will be generated
    // 01 Generate the EPWMxSOCB pulse on the first event: ETPS[SOCBCNT] = 0,1
    // 10 Generate the EPWMxSOCB pulse on the second event: ETPS[SOCBCNT] = 1,0
    // 11 Generate the EPWMxSOCB pulse on the third event: ETPS[SOCBCNT] = 1,1
    EPwm1Regs.ETPS.bit.SOCBPRD = 0;  

    //同SOCB
    EPwm1Regs.ETPS.bit.SOCACNT = 0;
    EPwm1Regs.ETPS.bit.SOCAPRD = 0; 

    //ePWM Interrupt Event (EPWMx_INT) Counter Register
    EPwm1Regs.ETPS.bit.INTCNT = 0;
    //ePWM Interrupt Event (EPWMx_INT) Counter Register
    EPwm1Regs.ETPS.bit.INTPRD = 0;  

    //////////////////////////////////////////////////////////////////////////////////
    //Event-Trigger Flag Register (ETFLG)SOC/INT事件发生标志位
    ////////////////////////////////////////////////////////////////////////////////////
    // Latched ePWM ADC Start-of-Conversion B (EPWMxSOCB) Status Flag
    // 0 Indicates no EPWMxSOCB event occurred
    // 1 Indicates that a start of conversion pulse was generated on EPWMxSOCB. The EPWMxSOCB 
    // output will continue to be generated even if the flag bit is set.
    if(EPwm1Regs.ETFLG.bit.SOCB == 0);
    if(EPwm1Regs.ETFLG.bit.SOCA == 0);
    if(EPwm1Regs.ETFLG.bit.INT == 0);
    //////////////////////////////////////////////////////////////////////////////////
    //Event-Trigger Flag Register (ETFLG)SOC/INT事件发生标志位清除（写1）
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.ETFLG.bit.SOCB = 0;
    EPwm1Regs.ETFLG.bit.SOCA = 0;    
    EPwm1Regs.ETFLG.bit.INT = 0;
    //////////////////////////////////////////////////////////////////////////////////
    //Event-Trigger Force Register (ETFRC)SOC/INT事件发生标志位 软件强制置位
    ////////////////////////////////////////////////////////////////////////////////////
    EPwm1Regs.ETFRC.bit.SOCB = 0;
    EPwm1Regs.ETFRC.bit.SOCA = 0;    
    EPwm1Regs.ETFRC.bit.INT = 0;









    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Trip Select (DCTRIPSEL) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    // Digital Compare B Low Input Select
    // Defines the source for the DCBL input. The TZ signals, when used as trip signals, are treated as 
    // normal inputs and can be defined as active high or active low.
    // 0000 TZ1 input
    // 0001 TZ2 input
    // 0010 TZ3 input
    // 1000 COMP1OUT input
    // 1001 COMP2OUT input
    // 1010 COMP3OUT input
    EPwm1Regs.DCTRIPSEL.bit.DCBLCOMPSEL = 0;
    // Digital Compare B high Input Select
    EPwm1Regs.DCTRIPSEL.bit.DCBHCOMPSEL = 0; 
    // Digital Compare A high Input Select
    EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 0; 
    // Digital Compare A low Input Select
    EPwm1Regs.DCTRIPSEL.bit.DCALCOMPSEL = 0; 

    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare A Control (DCACTL) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    // DCAEVT2 Force Synchronization Signal Select
    // 0 Source Is Synchronous Signal
    // 1 Source Is Asynchronous Signal
    EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = 0;
    // DCAEVT2 Source Signal Select
    // 0 Source Is DCAEVT2 Signal
    // 1 Source Is DCEVTFILT Signal
    EPwm1Regs.DCACTL.bit.EVT2SRCSEL = 0;
    // DCAEVT1 SYNC, Enable/Disable
    // 0 SYNC Generation Disabled
    // 1 SYNC Generation Enabled
    EPwm1Regs.DCACTL.bit.EVT1SYNCE = 0;
    // DCAEVT1 SOC, Enable/Disable
    // 0 SOC Generation Disabled
    // 1 SOC Generation Enabled
    EPwm1Regs.DCACTL.bit.EVT1SOCE = 0;
    //同EVT2
    EPwm1Regs.DCACTL.bit.EVT2FRCSYNCSEL = 0;
    //同EVT2
    EPwm1Regs.DCACTL.bit.EVT1SRCSEL = 0;
    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare B Control (DCBCTL) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    EPwm1Regs.DCBCTL.bit.EVT1FRCSYNCSEL = 0;
    EPwm1Regs.DCBCTL.bit.EVT2SRCSEL = 0;
    EPwm1Regs.DCBCTL.bit.EVT1SYNCE = 0;
    EPwm1Regs.DCBCTL.bit.EVT1SOCE = 0;
    EPwm1Regs.DCBCTL.bit.EVT2FRCSYNCSEL = 0;
    EPwm1Regs.DCBCTL.bit.EVT1SRCSEL = 0;

    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Filter Control (DCFCTL) Register  
    ////////////////////////////////////////////////////////////////////////////////////   
    // Pulse Select For Blanking & Capture Alignment
    // 00 Time-base counter equal to period (TBCTR = TBPRD)
    // 01 Time-base counter equal to zero (TBCTR = 0x0000)
    // 10 Reserved
    // 11 Reserved    
    EPwm1Regs.DCFCTL.bit.PULSESEL = 0;
    // Blanking Window Inversion
    // 0 Blanking window not inverted
    // 1 Blanking window inverted    
    EPwm1Regs.DCFCTL.bit.BLANKINV = 0;
    // Blanking Window Enable/Disable
    // 0 Blanking window is disabled
    // 1 Blanking window is enabled 
    EPwm1Regs.DCFCTL.bit.BLANKE = 0;
    //     Filter Block Signal Source Select
    // 00 Source Is DCAEVT1 Signal
    // 01 Source Is DCAEVT2 Signal
    // 10 Source Is DCBEVT1 Signal
    // 11 Source Is DCBEVT2 Signal   
    EPwm1Regs.DCFCTL.bit.SRCSEL = 0;

    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Capture Control (DCCAPCTL) Register 
    ////////////////////////////////////////////////////////////////////////////////////   
    // TBCTR Counter Capture Shadow Select Mode
    // 0 Enable shadow mode. The DCCAP active register is copied to shadow register on a TBCTR = 
    // TBPRD or TBCTR = zero event as defined by the DCFCTL[PULSESEL] bit. CPU reads of the 
    // DCCAP register will return the shadow register contents.
    // 1 Active Mode. In this mode the shadow register is disabled. CPU reads from the DCCAP register will 
    // always return the active register contents.    
    EPwm1Regs.DCCAPCTL.bit.SHDWMODE = 0;
    // TBCTR Counter Capture Enable/Disable
    // 0 Disable the time-base counter capture.
    // 1 Enable the time-base counter capture.    
    EPwm1Regs.DCCAPCTL.bit.CAPE = 0;

    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Filter Offset (DCFOFFSET) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    // Blanking Window Offset
    // These 16-bits specify the number of TBCLK cycles from the blanking window reference to the 
    // point when the blanking window is applied. The blanking window reference is either period or 
    // zero as defined by the DCFCTL[PULSESEL] bit.
    // This offset register is shadowed and the active register is loaded at the reference point defined 
    // by DCFCTL[PULSESEL]. The offset counter is also initialized and begins to count down when 
    // the active register is loaded. When the counter expires, the blanking window is applied. If the 
    // blanking window is currently active, then the blanking window counter is restarted    
    if(EPwm1Regs.DCFOFFSET == 100);
    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Filter Offset Counter (DCFOFFSETCNT) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    // Blanking Offset Counter
    // These 16-bits are read only and indicate the current value of the offset counter. The counter 
    // counts down to zero and then stops until it is re-loaded on the next period or zero event as 
    // defined by the DCFCTL[PULSESEL] bit.
    // The offset counter is not affected by the free/soft emulation bits. That is, it will always continue 
    // to count down if the device is halted by a emulation stop.
    if(EPwm1Regs.DCFOFFSETCNT == 100);
    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Filter Window (DCFWINDOW) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    EPwm1Regs.DCFWINDOW = 0;
    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Filter Window Counter (DCFWINDOWCNT) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    if(EPwm1Regs.DCFWINDOWCNT == 0);    
    //////////////////////////////////////////////////////////////////////////////////
    //Digital Compare Counter Capture (DCCAP) Register
    ////////////////////////////////////////////////////////////////////////////////////   
    if(EPwm1Regs.DCCAP == 0);    
}




/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
// InitEPwmGpio -  This function initializes GPIO pins to function as EPwm pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
void
InitEPwmGpio(void)
{
    #if DSP28_EPWM1
        InitEPwm1Gpio();
    #endif // endif DSP28_EPWM1
    #if DSP28_EPWM2
        InitEPwm2Gpio();
    #endif // endif DSP28_EPWM2
    #if DSP28_EPWM3
        InitEPwm3Gpio();
    #endif // endif DSP28_EPWM3
    #if DSP28_EPWM4
        InitEPwm4Gpio();
    #endif // endif DSP28_EPWM4
    #if DSP28_EPWM5
        InitEPwm5Gpio();
    #endif // endif DSP28_EPWM5
    #if DSP28_EPWM6
        InitEPwm6Gpio();
    #endif // endif DSP28_EPWM6
    #if DSP28_EPWM7
        InitEPwm7Gpio();
    #endif // endif DSP28_EPWM7
    #if DSP28_EPWM8
        InitEPwm8Gpio();
    #endif // endif DSP28_EPWM8
}

#if DSP28_EPWM1
//
// InitEPwm1Gpio - 
//
void
InitEPwm1Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    //
    // Configure EPWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM1 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}
#endif // endif DSP28_EPWM1

#if DSP28_EPWM2
//
// InitEPwm2Gpio - 
//
void
InitEPwm2Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)

    //
    // Configure EPwm-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM2 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}
#endif // endif DSP28_EPWM2

#if DSP28_EPWM3
//
// InitEPwm3Gpio -
//
void
InitEPwm3Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)

/* Configure EPwm-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}
#endif // endif DSP28_EPWM3

#if DSP28_EPWM4
void InitEPwm4Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)

    //
    // Configure EPWM-4 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM4 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
#endif // endif DSP28_EPWM4

#if DSP28_EPWM5
//
// InitEPwm5Gpio - 
//
void
InitEPwm5Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)

    //
    // Configure EPWM-5 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM5 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}
#endif // endif DSP28_EPWM5

#if DSP28_EPWM6
//
// InitEPwm6Gpio - 
//
void
InitEPwm6Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;   // Disable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;   // Disable pull-up on GPIO11 (EPWM6B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B

    EDIS;
}
#endif // endif DSP28_EPWM6

#if DSP28_EPWM7
//
// InitEPwm7Gpio - 
//
void
InitEPwm7Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO30 = 1; // Disable pull-up on GPIO30 (EPWM7A)
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;   // Disable pull-up on GPIO40 (EPWM7A)
    //GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1; // Disable pull-up on GPIO58 (EPWM7A)

    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;   // Disable pull-up on GPIO41 (EPWM7B)
    //GpioCtrlRegs.GPBPUD.bit.GPIO44 = 1; // Disable pull-up on GPIO44 (EPWM7B)

    //
    // Configure EPWM-7 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM7 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;   // Configure GPIO30 as EPWM7A
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 1;   // Configure GPIO40 as EPWM7A
    //GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;   // Configure GPIO58 as EPWM7A

    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 1;   // Configure GPIO41 as EPWM7B
    //GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;   // Configure GPIO44 as EPWM7B

    EDIS;
}
#endif // endif DSP28_EPWM7

#if DSP28_EPWM8
//
// InitEPwm8Gpio - 
//
void
InitEPwm8Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1; // Disable pull-up on GPIO30 (EPWM8A)
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 1;   // Disable pull-up on GPIO42 (EPWM8A)
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 1;   // Disable pull-up on GPIO43 (EPWM8B)

    //
    // Configure EPWM-7 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM7 functional 
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;   // Configure GPIO30 as EPWM8A
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 1;   // Configure GPIO42 as EPWM8A
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 1;   // Configure GPIO43 as EPWM8B

    EDIS;
}
#endif // endif DSP28_EPWM8

//
// InitEPwmSyncGpio - This function initializes GPIO pins to function as 
// EPwm Synch pins
//
void
InitEPwmSyncGpio(void)
{
    //   EALLOW;

    //
    // Configure EPWMSYNCI
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0; // Enable pull-up on GPIO6 (EPWMSYNCI)
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0; // Enable pull-up on GPIO32 (EPWMSYNCI)

    //
    // Set qualification for selected pins to asynch only
    // This will select synch to SYSCLKOUT for the selected pins.
    // Comment out other unwanted lines.
    //
    
    //
    // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
    //
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;
    
    //
    // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)
    //
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;

    //
    // Configure EPwmSync pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPwmSync 
    // functional pins.
    // Comment out other unwanted lines.
    //
    
    //
    // Configures GPIO6 for EPWMSYNCI operation
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;
    
    //
    // Configures GPIO32 for EPWMSYNCI operation.
    //
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;

    //
    // Configure EPWMSYNC0
    //

    //
    // Disable internal pull-up for the selected output pins for reduced power 
    // consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    //
    
    //
    // Disable pull-up on GPIO6 (EPWMSYNCO)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;
    
    //
    // Disable pull-up on GPIO33 (EPWMSYNCO)
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 1;

    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;   // Configures GPIO6 for EPWMSYNCO
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;  // Configures GPIO33 for EPWMSYNCO
}

//
// InitTzGpio - This function initializes GPIO pins to function as 
// Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
void
InitTzGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
    //GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;	  // Enable pull-up on GPIO42 (TZ1)
    //GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;	  // Enable pull-up on GPIO50 (TZ1)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
    //GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ2)
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ2)
    //GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;    // Enable pull-up on GPIO43 (TZ2)
    //GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;    // Enable pull-up on GPIO51 (TZ2)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
    //GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ3)
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ3)
    //GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;    // Enable pull-up on GPIO52 (TZ3)

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
    //GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = 3;  // Asynch input GPIO42 (TZ1)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3;  // Asynch input GPIO50 (TZ1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ2)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ2)
    //GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3;  // Asynch input GPIO43 (TZ2)
    //GpioCtrlRegs.GPBQAEL2.bit.GPIO51 = 3;  // Asynch input GPIO51 (TZ2)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ3)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ3)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 3;  // Asynch input GPIO52 (TZ3)

    //
    // Configure TZ pins using GPIO regs
    // This specifies which of the possible GPIO pins will be TZ functional 
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
    //GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 2;  // Configure GPIO42 as TZ1
    //GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 3;  // Configure GPIO50 as TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
    //GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ2
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ2
    //GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 2;  // Configure GPIO43 as TZ2
    //GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 3;  // Configure GPIO51 as TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
    //GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ3
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ3
    //GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 3;  // Configure GPIO52 as TZ3

    EDIS;
}

//
// End of file
//

