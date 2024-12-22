//###########################################################################
//
// FILE:    F2806x_Adc.c
//
// TITLE:   F2806x ADC Initialization & Support Functions.
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
// Defines
//
#define ADC_usDELAY  1000L
__interrupt void adc_isr(void);
//
// InitAdc - This function initializes ADC to a known state.
//
// NOTE: ADC INIT IS DIFFERENT ON F2806x DEVICES COMPARED TO OTHER 28X DEVICES
//
//  *IMPORTANT*
//  IF RUNNING FROM FLASH, PLEASE COPY OVER THE SECTION "ramfuncs"  FROM FLASH
//  TO RAM PRIOR TO CALLING InitSysCtrl(). THIS PREVENTS THE MCU FROM THROWING 
//  AN EXCEPTION WHEN A CALL TO DELAY_US() IS MADE. 
//
void
InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);

    //
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI
    // reserved OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs 
    // automatically in the Boot ROM. If the boot ROM code is bypassed during 
    // the debug process, the following function MUST be called for the ADC to 
    // function according to specification. The clocks to the ADC MUST be 
    // enabled before calling this function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.
    //
    EALLOW; 
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
    (*Device_cal)();


    EALLOW;
    PieVectTable.ADCINT1 = &adc_isr;
    //
    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, 
    // and ADC core. Before the first conversion is performed a 5ms delay must 
    // be observed after power up to give all analog circuits time to power up 
    // and settle
    //

    //
    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the F2806x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    //


    //
    //ADCCTL1 config
    //adc power control, and internal adc settings
    //
    AdcRegs.ADCCTL1.bit.RESET = 1;//reset ADC
    DELAY_US(10);
    /////////////////////////////////////////////////////////////////////////////////
    //AdcRegs.ADCCTL1.bit.ADCENABLE == 1;      //14 Enable ADC default 1 read only
    //AdcRegs.ADCCTL1.bit.ADCBSY == 1;          //adc busy
    //AdcRegs.ADCCTL1.bit.ADCBSYCHN == xx;          //adc busy
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      //7 Power ADC
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      //6 Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      //5 Power reference
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      //3 Select interal BG
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    //2 ADCINT1 trips after AdcResults latch
    AdcRegs.ADCCTL1.bit.VREFLOCONV = 0;     //1 VREFLO connection to ADCINB5 is disabled
    AdcRegs.ADCCTL1.bit.TEMPCONV = 0;     //ADCINA5 is passed to the ADC module as normal, internal temperature sensor connection to ADCINA5 is disabled.
    //ed




    //
    //ADCCTL2 config
    //clock deversion and overlap mode set
    //
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;
    AdcRegs.ADCCTL2.bit.CLKDIV4EN = 1;
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
    AdcOffsetSelfCal();
    EALLOW;
    //ed




    //
    //ADCINTFLGCLR config ADC Interrupt Flag Register
    //bits will be set when interrupt generated
    //
//  AdcRegs.ADCINTFLG.bit==1     //read only



    //
    //ADCINTFLGCLR config
    //set 1 to clr
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //ADC Interrupt Flag Clear




    //
    //ADCINTOVF ADC Interrupt Overflow Register
    //指示在产生ADCINT脉冲时是否发生了溢出现象。如果设置了相应的附加位并生成了选定的附加EOC触发器，则会发生溢出条件
    //AdcRegs.ADCINTOVF.bit.ADCINT1 == 1



    //
    //ADCINTOVFCLR config
    //
    AdcRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //ADC Interrupt Flag Clear



    //
    //interrupt Select Registers(INTSELxNy(INTSEL1N2,INTSEL3N4....)) config
    //adc interrupt settings
    //
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;  // Disable ADCINT1 Continuous mode 在用户清除ADCINTx标记（在ADCINTFLG寄存器中）之前，不会生成进一步的ADCINTx脉冲。
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;  //EOC1 is trigger for ADCINT1,the EOCx pulse will occur either at the beginning of a conversion or the end.





    //
    //SOCPRICTL config
    //ADC Start of Conversion Priority Control Register
    //
    AdcRegs.SOCPRICTL.bit.ONESHOT = 0; //oneshot disabled
    // AdcRegs.SOCPRICTL.bit.RRPOINTER==3 //read only
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0; //Determines the cutoff point for priority mode and round robin arbitration for SOCx



    //
    //ADCSAMPLEMODE config
    //
    AdcRegs.ADCSAMPLEMODE.all = 0; //Single sample mode set for all SOCx.



    //
    //ADC Interrupt Trigger SOC Select 1 Register (ADCINTSOCSEL1)
    //
    //SOCx ADC Interrupt Trigger Select. Select ADCINT to trigger SOCx. The ADCINT trigger is OR'ed
    //with the trigger selected by the TRIGSEL field in the ADCSOCxCTL register, as well as the
    //software force trigger signal from the ADCSOCFRC1 register.
    //00:No ADCINT will trigger SOCx.
    //01:ADCINT1 will trigger SOCx.
    //10:
    AdcRegs.ADCINTSOCSEL1.bit.SOC0 = 0;// dont use continus mode


     //
    //ADC SOC Flag 1 Register (ADCSOCFLG1)
    //当启动相应的SOCx转换时，该位将被自动清除。
    //AdcRegs.ADCSOCFLG1 == 1








    //
    //ADC SOC Force 1 Register (ADCSOCFRC1)
    //
    //SOCx Force Start of Conversion Flag. Writing a 1 will force to 1 the respective SOCx flag bit in the
    //ADCSOCFLG1 register. This can be used to initiate a software initiated conversion. Writes of 0 are ignored
    //0:No action
    //1:Force SOCx flag bit to 1. This will cause a conversion to start once priority is given to SOCx.
    //If software tries to set this bit on the same clock cycle that hardware tries to clear the SOCx bit in
    //the ADCSOCFLG1 register, then software has priority and the ADCSOCFLG1 bit will be set. In this
    //case the overflow bit in the ADCSOCOVF1 register will not be affected regardless of whether the
    //ADCSOCFLG1 bit was previously set or not.
    //
    AdcRegs.ADCSOCFRC1.all=0;




    //
    //ADC SOC Overflow 1 Register (ADCSOCOVF1)
    //
    //SOCx Start of Conversion Overflow Flag. Indicates an SOCx event was generated while an
    //existing SOCx event was already pending.(SOCx转换溢出标志。表示在现有的SOCx事件已经挂起时生成了SOCx事件。)
    //
    //0:No SOCx event overflow
    //1:SOCx event overflow
    //An overflow condition does not stop SOCx events from being processed. It simply is an indication
    //that a trigger was missed.
    //
    //read only
    //AdcRegs.ADCSOCOVF1




    //
    //ADC SOC Overflow Clear 1 Register (ADCSOCOVFCLR1)
    //
    //SOCx Clear Start of Conversion Overflow Flag. Writing a 1 will clear the respective SOCx overflow
    //flag in the ADCSOCOVF1 register. Writes of 0 are ignored.(SOCx清除转换溢出标志。写入一个1将清除ADCSOCOVF1寄存器中相应的SOCx溢出标志。忽略0的写入数)
    //0:No action
    //1:Clear SOCx overflow flag.
    //If software tries to set this bit on the same clock cycle that hardware tries to set the overflow bit in
    //the ADCSOCOVF1 register, then hardware has priority and the ADCSOCOVF1 bit will be set.
    //
    AdcRegs.ADCSOCOVFCLR1.bit.SOC0 = 1;




    //
    //ADC SOC0-SOC15 Control Registers (ADCSOCxCTL)
    //
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0;//SOCx Trigger Source Select.,  Software only
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x3F;//Sample window is 64 cycles long (63 + 1 clock cycles).
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0;//SOCx Trigger Source Select.,  Software only
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x3F;//Sample window is 64 cycles long (63 + 1 clock cycles).

    ////SOCx Channel Select. Selects the channel to be converted when SOCx is received by the ADC.
    //Sequential Sampling Mode (SIMULENx = 0)(就是SOCx选择使用哪个ADC通道)
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;  // set SOC1 channel select to ADCINA1



    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //

    DELAY_US(10);

    EDIS;

}
__interrupt void
adc_isr(void)
{

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

}
//
// InitAdcAio -
//
void
InitAdcAio()
{
    EALLOW;

    //
    // Configure ADC pins using AIO regs
    // This specifies which of the possible AIO pins will be Analog input pins.
    // NOTE: AIO1,3,5,7-9,11,13,15 are analog inputs in all AIOMUX1
    // configurations.
    // Comment out other unwanted lines.
    //

    //
    // Configure AIO2 for A2 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO2 = 2;

    //
    // Configure AIO4 for A4 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO4 = 2;

    //
    // Configure AIO6 for A6 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO6 = 2;

    //
    // Configure AIO10 for B2 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO10 = 2;

    //
    // Configure AIO12 for B4 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO12 = 2;

    //
    // Configure AIO14 for B6 (analog input) operation
    //
    GpioCtrlRegs.AIOMUX1.bit.AIO14 = 2;

    EDIS;
}

//
// AdcoffsetSelfCal - This function re-calibrates the ADC zero offset error by 
// converting the VREFLO reference with the ADC and modifying the ADCOFFTRIM 
// register. VREFLO is sampled by the ADC using an internal MUX select which 
// connects VREFLO to A5 without sacrificing an external ADC pin. This function
// calls two other functions:
// - AdcChanSelect(channel) � selects the ADC channel to convert
// - AdcConversion() � initiates several ADC conversions and returns the 
//   average
//
void
AdcOffsetSelfCal()
{
    Uint16 AdcConvMean;
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;   // Select internal reference mode
    
    //
    // Select VREFLO internal connection on B5
    //
    AdcRegs.ADCCTL1.bit.VREFLOCONV = 1;
    
    //
    // Select channel B5 for all SOC
    //
    AdcChanSelect(13);
    
    //
    // Apply artificial offset (+80) to account for a negative offset that may 
    // reside in the ADC core
    //
    AdcRegs.ADCOFFTRIM.bit.OFFTRIM = 80;
    
    AdcConvMean = AdcConversion();       // Capture ADC conversion on VREFLO
    
    //
    // Set offtrim register with new value (i.e remove artical offset (+80) 
    // and create a two's compliment of the offset error)
    //
    AdcRegs.ADCOFFTRIM.bit.OFFTRIM = 80 - AdcConvMean;
    
    //
    // Select external ADCIN5 input pin on B5
    //
    AdcRegs.ADCCTL1.bit.VREFLOCONV = 0;
    
    EDIS;
}

//
// AdcChanSelect - This function selects the ADC channel to convert by setting
// all SOC channel selects to a single channel.
// * IMPORTANT *
// This function will overwrite previous SOC channel select settings.
// Recommend saving the previous settings.
//
void
AdcChanSelect(Uint16 ch_no)
{
    AdcRegs.ADCSOC0CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC1CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC2CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC3CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC4CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC5CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC6CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC7CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC8CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC9CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC10CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC11CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC12CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC13CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC14CTL.bit.CHSEL= ch_no;
    AdcRegs.ADCSOC15CTL.bit.CHSEL= ch_no;
}

//
// AdcConversion - This function initiates several ADC conversions and returns
// the average. It uses ADCINT1 and ADCINT2 to "ping-pong" between SOC0-7
// and SOC8-15 and is referred to as "ping-pong" sampling.
// * IMPORTANT *
// This function will overwrite previous ADC settings. Recommend saving
// previous settings.
//
Uint16
AdcConversion(void)
{
    Uint16 index, SampleSize, Mean, ACQPS_Value;
    Uint32 Sum;

    index       = 0;            // initialize index to 0

    //
    // set sample size to 256
    // (**NOTE: Sample size must be multiples of 2^x where is an integer >= 4)
    //
    SampleSize  = 256;
    Sum         = 0;            // set sum to 0
    Mean        = 999;          // initialize mean to known value

    //
    // Set the ADC sample window to the desired value
    // (Sample window = ACQPS + 1)
    //
    ACQPS_Value = 6;
    AdcRegs.ADCSOC0CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC1CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC2CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC3CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC4CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC5CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC6CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC7CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC8CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC9CTL.bit.ACQPS  = ACQPS_Value;
    AdcRegs.ADCSOC10CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC11CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC12CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC13CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC14CTL.bit.ACQPS = ACQPS_Value;
    AdcRegs.ADCSOC15CTL.bit.ACQPS = ACQPS_Value;

    //
    // Enable ping-pong sampling
    //

    //
    // Enabled ADCINT1 and ADCINT2
    //
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    AdcRegs.INTSEL1N2.bit.INT2E = 1;

    //
    // Disable continuous sampling for ADCINT1 and ADCINT2
    //
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;
    AdcRegs.INTSEL1N2.bit.INT2CONT = 0;

    //
    // ADCINTs trigger at end of conversion
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Setup ADCINT1 and ADCINT2 trigger source
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL = 6;      // EOC6 triggers ADCINT1
    AdcRegs.INTSEL1N2.bit.INT2SEL = 14;     // EOC14 triggers ADCINT2

    //
    // Setup each SOC's ADCINT trigger source
    //
    AdcRegs.ADCINTSOCSEL1.bit.SOC0  = 2;    // ADCINT2 starts SOC0-7
    AdcRegs.ADCINTSOCSEL1.bit.SOC1  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC2  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC3  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC4  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC5  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC6  = 2;
    AdcRegs.ADCINTSOCSEL1.bit.SOC7  = 2;
    AdcRegs.ADCINTSOCSEL2.bit.SOC8  = 1;    // ADCINT1 starts SOC8-15
    AdcRegs.ADCINTSOCSEL2.bit.SOC9  = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC10 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC11 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC12 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC13 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC14 = 1;
    AdcRegs.ADCINTSOCSEL2.bit.SOC15 = 1;

    DELAY_US(ADC_usDELAY);              // Delay before converting ADC channels

    //
    // ADC Conversion
    //

    //
    // Force Start SOC0-7 to begin ping-pong sampling
    //
    AdcRegs.ADCSOCFRC1.all = 0x00FF;

    while( index < SampleSize )
    {
        //
        // Wait for ADCINT1 to trigger, then add ADCRESULT0-7 registers to sum
        //
        while (AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
        {

        }

        //
        // Must clear ADCINT1 flag since INT1CONT = 0
        //
        AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
        Sum += AdcResult.ADCRESULT0;
        Sum += AdcResult.ADCRESULT1;
        Sum += AdcResult.ADCRESULT2;
        Sum += AdcResult.ADCRESULT3;
        Sum += AdcResult.ADCRESULT4;
        Sum += AdcResult.ADCRESULT5;
        Sum += AdcResult.ADCRESULT6;

        //
        // Wait for SOC9 conversion to start, which gives time for SOC7
        // conversion result
        //
        while( AdcRegs.ADCSOCFLG1.bit.SOC9 == 1 )
        {

        }
        Sum += AdcResult.ADCRESULT7;

        //
        // Wait for ADCINT2 to trigger, then add ADCRESULT8-15 registers to sum
        //
        while (AdcRegs.ADCINTFLG.bit.ADCINT2 == 0)
        {

        }

        //
        // Must clear ADCINT2 flag since INT2CONT = 0
        //
        AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;

        Sum += AdcResult.ADCRESULT8;
        Sum += AdcResult.ADCRESULT9;
        Sum += AdcResult.ADCRESULT10;
        Sum += AdcResult.ADCRESULT11;
        Sum += AdcResult.ADCRESULT12;
        Sum += AdcResult.ADCRESULT13;
        Sum += AdcResult.ADCRESULT14;

        //
        // Wait for SOC1 conversion to start, which gives time for
        // SOC15 conversion result
        //
        while( AdcRegs.ADCSOCFLG1.bit.SOC1 == 1 )
        {

        }
        Sum += AdcResult.ADCRESULT15;

        index+=16;

    }

    //
    // Disable ADCINT1 and ADCINT2 to STOP the ping-pong sampling
    //
    AdcRegs.INTSEL1N2.bit.INT1E = 0;
    AdcRegs.INTSEL1N2.bit.INT2E = 0;

    //
    // Wait for any pending SOCs to complete
    //
    while(AdcRegs.ADCSOCFLG1.all != 0)
    {

    }

    //
    // Clear any pending interrupts
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
    AdcRegs.ADCINTOVFCLR.bit.ADCINT2 = 1;

    //
    // reset RR pointer to 32, so that next SOC is SOC0
    //
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while( AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1 );
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while( AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0 );

    Mean = Sum / SampleSize;    // Calculate average ADC sample value

    return Mean;                // return the average
}


//
// End of file
//

