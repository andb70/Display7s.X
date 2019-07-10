#include "shift595.h"
//#include <p32xxxx.h> // Include PIC32 specifics header file
#include <plib.h> // Include the PIC32 Peripheral Library
//#include <stdio.h>
//#include <stdlib.h>
// initialize ports used with display
void initShift595(){    
    if ( SHIFT_SER_PORT == IOPORT_B & SHIFT_SER_BIT == BIT_13 \
       | SHIFT_RCLK_PORT == IOPORT_B & SHIFT_RCLK_BIT == BIT_13 \
       | SHIFT_SRCLK_PORT == IOPORT_B & SHIFT_SRCLK_BIT == BIT_13 )
        mJTAGPortEnable(0); // Disable JTAG    
    PORTSetPinsDigitalOut(SHIFT_SER_PORT, SHIFT_SER_BIT);
    PORTSetPinsDigitalOut(SHIFT_RCLK_PORT, SHIFT_RCLK_BIT);
    PORTSetPinsDigitalOut(SHIFT_SRCLK_PORT, SHIFT_SRCLK_BIT);
#ifdef SHIFT_SRCLR_EN
    if ( SHIFT_SRCLR_PORT == IOPORT_B & SHIFT_SRCLR_BIT == BIT_13 )
        mJTAGPortEnable(0); // Disable JTAG    
    PORTSetPinsDigitalOut(SHIFT_SRCLR_PORT, SHIFT_SRCLR_BIT);
#endif
}
char shiftdata595(unsigned char data) {
//  send data with serial communication to the shifter
//  return:
//      0 during job
//      1 when done
//
//        INPUTS            FUNCTION
//  SER	SR	SR	R	
//  SER	CLK	CLR	CLK	OE
//  X	R	H	 	        First stage of the shift register goes X.
//  ________________________Other stages store the data of previous stage, respectively.
//   	 	 	R	        Shift-register data is stored in the storage register.   
// 
// H:   High
// R:   Raising edge
// X:   Low or High

    
    // shift program state
    static enum _shiftRegisterConstants{
        srcReadInput,
        srcLoadShiftRegister,
        srcWriteToOutput,
    } sState;
    
    // buffer shifting 
    static unsigned char buffer;
    static unsigned char position;
    char returnCode=0;
    
    switch (sState)
    {
        case srcReadInput:
            buffer = data;
            SHIFT_RCLK = 0; 
            position = 7;
            sState=srcLoadShiftRegister;
            break;
            
        case srcLoadShiftRegister:
            SHIFT_SER = (buffer >> position) & 0x01;
            SHIFT_SRCLK = 1;
            sState=srcWriteToOutput;
            break;
            
        case srcWriteToOutput:
            SHIFT_SRCLK = 0;
            //if (i++ < 7)
            if (position-- > 0)
            {
                sState = srcLoadShiftRegister;
                break;
            }
            SHIFT_RCLK = 1;
            sState = srcReadInput;
            returnCode = 1;
            break;
    }
    return returnCode;
}
