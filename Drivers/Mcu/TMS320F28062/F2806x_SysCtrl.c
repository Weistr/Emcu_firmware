//###########################################################################
//
// FILE:   F2806x_SysCtrl.c
//
// TITLE:  F2806x Device System Control Initialization & Support Functions.
//
// DESCRIPTION:
//
//         Example initialization of system resources.
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
#include "F2806x_Device.h"     // Headerfile Include File
#include "F2806x_Examples.h"   // Examples Include File

//
// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped to a load and
// run address using the linker cmd file.
//
//  *IMPORTANT*
//  IF RUNNING FROM FLASH, PLEASE COPY OVER THE SECTION "ramfuncs"  FROM FLASH
//  TO RAM PRIOR TO CALLING InitSysCtrl(). THIS PREVENTS THE MCU FROM THROWING 
//  AN EXCEPTION WHEN A CALL TO DELAY_US() IS MADE. 
//
#pragma CODE_SECTION(InitFlash, "ramfuncs");

//
// InitSysCtrl - This function initializes the System Control registers to a 
// known state.
// - Disables the watchdog
// - Set the PLLCR for proper SYSCLKOUT frequency
// - Set the pre-scaler for the high and low frequency peripheral clocks
// - Enable the clocks to the peripherals
//
void
InitSysCtrl(void)
{
    //
    // Disable the watchdog
    //
    DisableDog();

    //
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC & oscillator calibration 
    // values from TI reserved OTP into the appropriate trim registers, occurs 
    // automatically in the Boot ROM. If the boot ROM code is bypassed during 
    // the debug process, the following function MUST be called for the ADC and
    // oscillators to function according to specification. The clocks to the 
    // ADC MUST be enabled before calling this function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
    (*Device_cal)();
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
    EDIS;

    //
    // Select Internal Oscillator 1 as Clock Source (default), and turn off all
    // unused clocks to conserve power.
    //
    //IntOsc1Sel();//使用内部时钟
    XtalOscSel();//使用外部晶振
    //
    // Initialize the PLL control: PLLCR and CLKINDIV
    // DSP28_PLLCR and DSP28_CLKINDIV are defined in F2806x_Examples.h
    //
    InitPll(DSP28_PLLCR,DSP28_DIVSEL);

    //
    // Initialize the peripheral clocks
    //
    InitPeripheralClocks();
}

//
// InitFlash - This function initializes the Flash Control registers for 
// operation at 90MHz.  If you are running slower flash wait states can be 
// lessened. Refer to the datasheet for propper flash wait state values.
//                   CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results
//
void
InitFlash(void)
{
    EALLOW;
    
    //
    // Enable Flash Pipeline mode to improve performance of code executed from 
    // Flash.
    //
    FlashRegs.FOPT.bit.ENPIPE = 1;

    //                CAUTION
    // Minimum waitstates required for the flash operating
    // at a given CPU rate must be characterized by TI.
    // Refer to the datasheet for the latest information.
    //
    
    //
    // Set the Paged Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3;

    //
    // Set the Random Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.RANDWAIT = 3;

    //
    // Set the Waitstate for the OTP
    //
    FlashRegs.FOTPWAIT.bit.OTPWAIT = 5;

    //
    // CAUTION -  ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
    //
    FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
    FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
    EDIS;
 
    //
    // Force a pipeline flush to ensure that the write to the last register
    // configured occurs before returning.
    //
    __asm(" RPT #7 || NOP");
}

//
// ServiceDog - This function resets the watchdog timer. Enable this function 
// for using ServiceDog in the application
//
void
ServiceDog(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;
}

//
// DisableDog - This function disables the watchdog timer.
//
void
DisableDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

//
// InitPll - This function initializes the PLLCR register.
//
void
InitPll(Uint16 val, Uint16 divsel)
{
    volatile Uint16 iVol;

    // Make sure the PLL is not running in limp mode
    if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
    {
    EALLOW;
    // OSCCLKSRC1 failure detected. PLL running in limp mode.
    // Re-enable missing clock logic.
    SysCtrlRegs.PLLSTS.bit.MCLKCLR = 1;
    EDIS;
    // Replace this line with a call to an appropriate
    // SystemShutdown(); function.
    __asm("        ESTOP0");     // Uncomment for debugging purposes
    }

    // DIVSEL MUST be 0 before PLLCR can be changed from
    // 0x0000. It is set to 0 by an external reset XRSn
    // This puts us in 1/4
    if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
    {
    EALLOW;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
    EDIS;
    }

    //
    // Change the PLLCR
    //
    if (SysCtrlRegs.PLLCR.bit.DIV != val)
    {
        EALLOW;
        
        //
        // Before setting PLLCR turn off missing clock detect logic
        //
        SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
        SysCtrlRegs.PLLCR.bit.DIV = val;
        EDIS;
        
        //
        // Optional: Wait for PLL to lock.
        // During this time the CPU will switch to OSCCLK/2 until
        // the PLL is stable.  Once the PLL is stable the CPU will
        // switch to the new PLL value.
        //
        // This time-to-lock is monitored by a PLL lock counter.
        //
        // Code is not required to sit and wait for the PLL to lock.
        // However, if the code does anything that is timing critical,
        // and requires the correct clock be locked, then it is best to
        // wait until this switching has completed.
        //

        //
        // Wait for the PLL lock bit to be set.
        //

        //
        // The watchdog should be disabled before this loop, or fed within
        // the loop via ServiceDog().
        //
        
        //
        // Uncomment to disable the watchdog
        //
        DisableDog();

        while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
        {
            //
            // Uncomment to service the watchdog
            //
            //ServiceDog();
        }

        EALLOW;
        SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
        EDIS;
    }

    //
    // If switching to 1/2
    //
    if((divsel == 1)||(divsel == 2))
    {
        EALLOW;
        SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
        EDIS;
    }

    //
    // If switching to 1/1
    // * First go to 1/2 and let the power settle
    //   The time required will depend on the system, this is only an example
    // * Then switch to 1/1
    //
    if(divsel == 3)
    {
        EALLOW;
        SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
        DELAY_US(50L);
        SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
        EDIS;
    }
}

//
// InitPll2 - This function initializes the PLL2 registers.
//
void
InitPll2(Uint16 clksrc, Uint16 pllmult, Uint16 clkdiv)
{
    EALLOW;

    //
    // Check if SYSCLK2DIV2DIS is in /2 mode
    //
    if(DevEmuRegs.DEVICECNF.bit.SYSCLK2DIV2DIS != 0)
    {
        DevEmuRegs.DEVICECNF.bit.SYSCLK2DIV2DIS = 0;
    }

    //
    // Enable PLL2
    //
    SysCtrlRegs.PLL2CTL.bit.PLL2EN = 1;
    
    //
    // Select clock source for PLL2
    //
    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = clksrc;
    
    //
    // Set PLL2 Multiplier
    //
    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = pllmult;
    
    //
    // Wait for PLL to lock.
    // Uncomment to disable the watchdog
    //
    DisableDog();
    while(SysCtrlRegs.PLL2STS.bit.PLL2LOCKS!= 1)
    {
        //
        // Uncomment to service the watchdog
        //
        //ServiceDog();
    }

    //
    // Set System Clock 2 divider
    //
    DevEmuRegs.DEVICECNF.bit.SYSCLK2DIV2DIS = clkdiv;
    EDIS;
}

//
// InitPeripheralClocks - This function initializes the clocks to the 
// peripheral modules. First the high and low clock prescalers are set
// Second the clocks are enabled to each peripheral.
// To reduce power, leave clocks to unused peripherals disabled
//
// Note: If a peripherals clock is not enabled then you cannot
// read or write to the registers for that peripheral
//
void
InitPeripheralClocks(void)
{
    EALLOW;
    SysCtrlRegs.LOSPCP.all = 0x0002;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;      // ADC
    SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

    EDIS;
}

//
// CsmUnlock - This function unlocks the CSM. User must replace 0xFFFF's with 
// current password for the DSP. Returns 1 if unlock is successful.
//
#define STATUS_FAIL          0
#define STATUS_SUCCESS       1
Uint16
CsmUnlock()
{
    volatile Uint16 temp;

    //
    // Load the key registers with the current password. The 0xFFFF's are dummy
    // passwords.  User should replace them with the correct password for the 
    // DSP.
    //
    EALLOW;
    CsmRegs.KEY0 = 0xFFFF;
    CsmRegs.KEY1 = 0xFFFF;
    CsmRegs.KEY2 = 0xFFFF;
    CsmRegs.KEY3 = 0xFFFF;
    CsmRegs.KEY4 = 0xFFFF;
    CsmRegs.KEY5 = 0xFFFF;
    CsmRegs.KEY6 = 0xFFFF;
    CsmRegs.KEY7 = 0xFFFF;
    EDIS;

    //
    // Perform a dummy read of the password locations if they match the key 
    // values, the CSM will unlock
    //
    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    //
    // If the CSM unlocked, return succes, otherwise return
    // failure.
    //
    if (CsmRegs.CSMSCR.bit.SECURE == 0)
    {
        return STATUS_SUCCESS;
    }
    else
    {
        return STATUS_FAIL;
    }
}

//
// IntOsc1Sel - This function switches to Internal Oscillator 1 and turns off 
// all other clock sources to minimize power consumption
// 关于时钟输入，参考技术手册p69
void
IntOsc1Sel(void)
{
    EALLOW;
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0; //使能内部时钟1（内部RC时钟）
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1，选择内部时钟1而不是2
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN，关闭外部时钟
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC，关闭晶振振荡器
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2，关闭内部时钟2
    EDIS;
}

//
// IntOsc2Sel - This function switches to Internal oscillator 2 from External 
// Oscillator and turns off all other clock sources to minimize 
// power consumption
// NOTE: If there is no external clock connection, when switching from
//       INTOSC1 to INTOSC2, EXTOSC and XLCKIN must be turned OFF prior
//       to switching to internal oscillator 1
//
void
IntOsc2Sel(void)
{
    EALLOW;
    
    //
    // Errata Workaround - Toggle XCLKINOFF and XTALOSCOFF bits
    //
    SysCtrlRegs.CLKCTL.all |= 0x6000;
    SysCtrlRegs.CLKCTL.all &= ~0x6000;
    SysCtrlRegs.CLKCTL.all |= 0x6000;
    SysCtrlRegs.CLKCTL.all &= ~0x6000;    
    SysCtrlRegs.CLKCTL.all |= 0x6000;
    
    //
    // Turn on INTOSC2, set as source
    //
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 0;     // Turn on INTOSC2
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 1;  // Select INTOSC2 as source
    
    //
    // Switch to use Internal Oscillator 2
    //
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;
    
    //
    // Clock Watchdog off of INTOSC1 always
    //
    SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 0;
    
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
    EDIS;
}

//
// XtalOscSel - This function switches to External CRYSTAL oscillator and turns
// off all other clock sources to minimize power consumption. This option may 
// not be available on all device packages
//
void
XtalOscSel(void)
{
    EALLOW;
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC
    
    //
    // Wait for 1ms while XTAL starts up
    //
    DELAY_US(1000);
    
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  // Switch to external clock
    
    //
    // Switch from INTOSC1 to INTOSC2/ext clk
    //
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;   
    
    //
    // Clock Watchdog off of INTOSC1 always
    //
    SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 0;
    
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;     // Turn off INTOSC2
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
    EDIS;
}

//
// ExtOscSel - This function switches to External oscillator and turns off all 
// other clock sources to minimize power consumption.
//
void
ExtOscSel(void)
{
    EALLOW;
    
    //
    // 1-GPIO19 = XCLKIN, 0-GPIO38 = XCLKIN
    //
    SysCtrlRegs.XCLK.bit.XCLKINSEL = 1;       
    
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 1;    // Turn on XTALOSC
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 0;     // Turn on XCLKIN
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0; // Switch to external clock
    
    //
    // Switch from INTOSC1 to INTOSC2/ext clk
    //
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;
    
    //
    // Clock Watchdog off of INTOSC1 always
    //
    SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 0;   
    
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;    // Turn off INTOSC2
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;    // Leave INTOSC1 on
    EDIS;
}

//
// End of File
//

