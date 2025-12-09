/*
 * board.c
 *
 *  Created on:
 *      Author: 86133
 */
#include "board.h"
uint8_t flg1 = 0,flg2 = 0;
//20ms
void boardApp(void)
{
    GpioDataRegs.GPBTOGGLE.bit.GPIO53 = 1;
    EALLOW;
    AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;
    AdcRegs.ADCSOCFRC1.bit.SOC1 = 1;


    EDIS;
}

void boardInit(void)
{
    //
    //Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    //  Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();
    ///////////////////////////////////////////////////////////////////////
    InitGpio();
    InitCpuTimers();

    InitAdc();
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER |= M_INT1;                     // Enable CPU Interrupt 1
    ERTM;                              // Enable Global realtime interrupt DBGM
    //使能全局中断
    EINT;
}

