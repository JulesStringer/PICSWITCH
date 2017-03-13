/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

    // Define a macro describing this hardware set up (used in other files)
#ifndef LIGHT_SITE
#define LIGHT_SITE
#endif

// Indicates that RTS/CTS protocol is to be used with multiplexer
#define USE_RTSCTS 0
// Set configuration fuses (but only in MainDemo.c where THIS_IS_STACK_APPLICATION is defined)
#if defined(THIS_IS_STACK_APPLICATION)
//        #pragma config WDTEN=OFF, OSC=INTOSCPLL, CFGPLLEN=ON, PLLSEL=PLL96
        #pragma config WDTEN=OFF, OSC=INTOSC, CFGPLLEN=OFF

	// Automatically set Extended Instruction Set fuse based on compiler setting
	#if defined(__EXTENDED18__)
		#pragma config XINST=ON
	#else
		#pragma config XINST=OFF
	#endif
#endif

#undef MPFS_USE_EEPROM
//#define MPFS_USE_SPI_FLASH
// Clock frequency values
// These directly influence timed events using the Tick module.  They also are used for UART and SPI baud rate generation.
#ifdef SYSTEM_CLOCK_4
#define GetSystemClock()		(4000000ul)			// Hz
#define IRCF_VALUE 0x5
//#define TICKS_PER_SECOND 7
#endif
#ifdef SYSTEM_CLOCK_16
#define GetSystemClock()		(16000000ul)			// Hz
#define IRCF_VALUE 0x7
//#define TICKS_PER_SECOND 30
// 16MHZ - allows time to service the light and received characters
#endif
//#define GetSystemClock()       (48000000ul)
#define GetInstructionClock()	(GetSystemClock()/4)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Might need changing if using Doze modes.
#define GetPeripheralClock()	(GetSystemClock()/4)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Divisor may be different if using a PIC32 since it's configurable.

#define TMR_PRESCALER 256
#define TICKS_PER_SECOND ( (GetPeripheralClock() + TMR_PRESCALER/2) / TMR_PRESCALER)

// Hardware I/O pin mappings

// TX is on pin 17
#define TX_ANSEL                   (ANSELCbits.ANSC6)

// RX is on pin 18
#define RX_ANSEL                   (ANSELCbits.ANSC7)
#define RX_TRIS                    (TRISCbits.TRISC7)

// DI is on pin 15

#define DI_ANSEL                   (PRODL)
#define DI_TRIS                    (PRODL)                  
#define DI_IO                      (PRODL)
#define DI_PORT                    (PRODL)
//#define NO_UART1 0

// DI2 is nul as it is stright duplex serial port
#define DI2_ANSEL                  (PRODL)
#define DI2_TRIS                   (PRODL)
#define DI2_IO                     (PRODL)
#define DI2_PORT                   (PRODL)

// Logging port reuses pins 27 and 28
#define RX2_ANSEL                  (PRODL)
#define RX2_TRIS                   (TRISBbits.TRISB7)

#define TX2_ANSEL                  (PRODL)
#define TX2_IO                     (LATBbits.LATB6)
#define TX2_TRIS                   (TRISBbits.TRISB6)

#define LED_COUNT 4
#define MAX_MANUAL_PWM 0
#undef MANAUL_PWM_USE_TIMER

#define CHANNEL_0_A_TRIS           (TRISCbits.TRISC0)
#define CHANNEL_0_A_LAT            (LATCbits.LATC0)
#define CHANNEL_0_A_ANSEL          (PRODL)

#define CHANNEL_0_B_TRIS           (TRISCbits.TRISC1)
#define CHANNEL_0_B_LAT            (LATCbits.LATC1)
#define CHANNEL_0_B_ANSEL          (PRODL)
    
#define CHANNEL_1_A_TRIS           (TRISCbits.TRISC2)
#define CHANNEL_1_A_LAT            (LATCbits.LATC2)
#define CHANNEL_1_A_ANSEL          (ANSELCbits.ANSC2)

#define CHANNEL_1_B_TRIS           (TRISCbits.TRISC2)
#define CHANNEL_1_B_LAT            (LATCbits.LATC2)
#define CHANNEL_1_B_ANSEL          (ANSELCbits.ANSC2)

#define CHANNEL_0_SW_TRIS          (TRISAbits.TRISA4)    
#define CHANNEL_0_SW_ANSEL         (PRODL)    
#define CHANNEL_0_SW_LAT           (LATAbits.LATA4)    

#define CHANNEL_1_SW_TRIS          (TRISAbits.TRISA5)    
#define CHANNEL_1_SW_ANSEL         (ANSELAbits.ANSA5)    
#define CHANNEL_1_SW_LAT           (LATAbits.LATA5)    

#define VOLUME_ADR_0_ANSEL         (ANSELBbits.ANSB5)
#define VOLUME_ADR_0_TRIS          (TRISBbits.TRISB5)
#define VOLUME_ADR_0_LAT           (LATBbits.LATB5)

#define VOLUME_ADR_1_ANSEL         (ANSELBbits.ANSB4)
#define VOLUME_ADR_1_TRIS          (TRISBbits.TRISB4)
#define VOLUME_ADR_1_LAT           (LATBbits.LATB4)

#define VOLUME_RESET_ANSEL         (ANSELBbits.ANSB3)
#define VOLUME_RESET_TRIS          (TRISBbits.TRISB3)
#define VOLUME_RESET_LAT           (LATBbits.LATB3)

#define VOLUME_SDA_ANSEL         (ANSELBbits.ANSB2)
#define VOLUME_SDA_TRIS          (TRISBbits.TRISB2)
#define VOLUME_SCL_ANSEL         (ANSELBbits.ANSB1)
#define VOLUME_SCL_TRIS          (TRISBbits.TRISB1)

#define VOLUME_ADDR              0x26
    // At POR and before output from PIC is A0 high or low? - does this need an external pullup / down?
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

