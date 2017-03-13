/*********************************************************************
 *
 *	Hardware specific definitions for:
 *    - PICDEM.net 2
 *    - PIC18F97J60
 *    - Internal 10BaseT Ethernet
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    Compiler.h
 * Processor:       PIC18
 * Compiler:        Microchip C18 v3.36 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		09/16/2010	Regenerated for specific boards
 ********************************************************************/
#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include "Compiler.h"
#include "BoardDef.h"

// Define a macro describing this hardware set up (used in other files)
#ifndef LIGHTCONTROLLER_SITE
#define LIGHTCONTROLLER_SITE
#endif

// Set configuration fuses (but only in MainDemo.c where THIS_IS_STACK_APPLICATION is defined)
#if 0
//#if defined(THIS_IS_STACK_APPLICATION)
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

// TX2 is on pin 27
#define TX2_ANSEL                  (PRODL)

// RX2 is on pin 28
#define RX2_ANSEL                  (PRODL)
#define RX2_TRIS                   (TRISBbits.TRISB7)

// DI is on pin 16

#define DI_ANSEL                   (ANSELCbits.ANSC5)
#define DI_TRIS                    (TRISCbits.TRISC5)
#define DI_IO                      (LATCbits.LATC5)
#define DI_PORT                    (PORTCbits.RC5)

#define DI2_ANSEL                   (PRODL)
#define DI2_TRIS                    (PRODL)
#define DI2_IO                      (PRODL)
#define DI2_PORT                    (PRODL)
// Outputs and associated ADC
// LED1 pin 14
#define LED1_TRIS                  (TRISCbits.TRISC3)
#define LED1_LAT                   (LATCbits.LATC3)
#define LED1_PORT                  (PORTCbits.RC3)
#define LED1_ANSEL                 (ANSELCbits.ANSC3)
#define LED1_USE_PWM 0
#define LED1_PWM_TRIS LED1_TRIS
#define LED1_TSEL                     (PRODL)

// ADC1 pin 13
#define ADC1_ANSEL                 (ANSELCbits.ANSC2)
#define ADC1_TRIS                  (TRISCbits.TRISC2)
#define ADC1_CHS                   14

// LED2 pin 12
#define LED2_TRIS                  (TRISCbits.TRISC1)
#define LED2_LAT                   (LATCbits.LATC1)
#define LED2_PORT                  (PORTCbits.RC1)
#define LED2_ANSEL                 (PRODL)
#define LED2_USE_PWM 0
#define LED2_PWM_TRIS LED2_TRIS
#define LED2_TSEL                     (PRODL)

// ADC2 pin 7
#define ADC2_ANSEL                 (ANSELAbits.ANSA5)
#define ADC2_TRIS                  (TRISAbits.TRISA5)
#define ADC2_CHS                   4

// LED3 pin 6
#define LED3_TRIS                  (TRISAbits.TRISA4)
#define LED3_LAT                   (LATAbits.LATA4)
#define LED3_PORT                  (PORTAbits.RA4)
#define LED3_ANSEL                 (PRODL)
#define LED3_USE_PWM 0
#define LED3_PWM_TRIS LED3_TRIS
#define LED3_TSEL                     (PRODL)

// ADC3 pin 5
#define ADC3_ANSEL                 (ANSELAbits.ANSA3)
#define ADC3_TRIS                  (TRISAbits.TRISA3)
#define ADC3_CHS                   3

// LED4 pin 4
#define LED4_TRIS                  (TRISAbits.TRISA2)
#define LED4_LAT                   (LATAbits.LATA2)
#define LED4_PORT                  (PORTAbits.RA2)
#define LED4_ANSEL                 (ANSELAbits.ANSA2)
#define LED4_USE_PWM 0
#define LED4_PWM_TRIS LED4_TRIS
#define LED4_TSEL                     (PRODL)

// ADC4 pin 3
#define ADC4_ANSEL                 (ANSELAbits.ANSA1)
#define ADC4_TRIS                  (TRISAbits.TRISA1)
#define ADC4_CHS                   1

// LED5 pin 2
#define LED5_TRIS                  (TRISAbits.TRISA0)
#define LED5_LAT                   (LATAbits.LATA0)
#define LED5_PORT                  (PORTAbits.RA0)
#define LED5_ANSEL                 (ANSELAbits.ANSA0)
#define LED5_USE_PWM 0
#define LED5_PWM_TRIS LED5_TRIS
#define LED5_TSEL                     (PRODL)

// ADC5 pin 26
#define ADC5_ANSEL                 (ANSELBbits.ANSB5)
#define ADC5_TRIS                  (TRISBbits.TRISB5)
#define ADC5_CHS                   13

// LED6 pin 25
#define LED6_TRIS                  (TRISBbits.TRISB4)
#define LED6_LAT                   (LATBbits.LATB4)
#define LED6_PORT                  (PORTBbits.RB4)
#define LED6_ANSEL                 (ANSELBbits.ANSB4)
#define LED6_USE_PWM 0
#define LED6_PWM_TRIS LED6_TRIS
#define LED6_TSEL                     (PRODL)

// ADC6 pin 24
#define ADC6_ANSEL                 (ANSELBbits.ANSB3)
#define ADC6_TRIS                  (TRISBbits.TRISB3)
#define ADC6_CHS                   9

// LED7 pin 23
#define LED7_TRIS                  (TRISBbits.TRISB2)
#define LED7_LAT                   (LATBbits.LATB2)
#define LED7_PORT                  (PORTBbits.RB2)
#define LED7_ANSEL                 (ANSELBbits.ANSB2)
#define LED7_USE_PWM 0
#define LED7_PWM_TRIS LED7_TRIS
#define LED7_TSEL                     (PRODL)

// ADC7 pin 22
#define ADC7_ANSEL                 (ANSELBbits.ANSB1)
#define ADC7_TRIS                  (TRISBbits.TRISB1)
#define ADC7_CHS                   10

// LED8 pin 21
#define LED8_TRIS                  (TRISBbits.TRISB0)
#define LED8_LAT                   (LATBbits.LATB0)
#define LED8_PORT                  (PORTBbits.RB0)
#define LED8_ANSEL                 (ANSELBbits.ANSB0)
#define LED8_USE_PWM 0
#define LED8_PWM_TRIS LED8_TRIS
#define LED8_TSEL                     (PRODL)

// ADC8 pin 15
#define ADC8_ANSEL                 (ANSELCbits.ANSC4)
#define ADC8_TRIS                  (TRISCbits.TRISC4)
#define ADC8_CHS                   16

#define LED_COUNT 8
#define MIN_MANUAL_PWM 0
//#define ADCON2             ADCON1
// UART mapping functions for consistent API names across 8-bit and 16 or 
// 32 bit compilers.  For simplicity, everything will use "UART" instead 
// of USART/EUSART/etc.
#define BusyUART()			BusyUSART()
#define CloseUART()			CloseUSART()
#define ConfigIntUART(a)	ConfigIntUSART(a)
#define DataRdyUART()		DataRdyUSART()
#define OpenUART(a,b,c)		OpenUSART(a,b,c)
#define ReadUART()			ReadUSART()
#define WriteUART(a)		WriteUSART(a)
#define getsUART(a,b,c)		getsUSART(b,a)
#define putsUART(a)			putsUSART(a)
#define getcUART()			ReadUSART()
#define putcUART(a)			WriteUSART(a)
#define putrsUART(a)		putrsUSART((far rom char*)a)

#endif // #ifndef HARDWARE_PROFILE_H
