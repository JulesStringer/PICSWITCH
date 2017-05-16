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

#include <xc.h>
//#define AMPLIFIER 0 // temporary
#ifdef AMPLIFIER
#define LED_USE_PWM 0
#define LIGHT_CONTROLLER 0
#else
#define LED_USE_PWM 1
#endif

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

#define SWITCH_COUNT 0

#define LED_COUNT 16

// LED1 is pin 2
#define LED1_USE_PWM 3
#define LED1_ANSEL                    (ANSELAbits.ANSA0)
#define LED1_PWM_TRIS                 (TRISAbits.TRISA0)
#define LED1_TRIS LED1_PWM_TRIS
#define LED1_LAT                      (LATAbits.LATA0)
#define LED1_PORT                     (PORTAbits.RA0)
#define LED1_TSEL                     (PRODL)
// LED2 is pin 4
#define LED2_USE_PWM 3
#define LED2_ANSEL                    (ANSELAbits.ANSA2)
#define LED2_PWM_TRIS                 (TRISAbits.TRISA2)
#define LED2_TRIS LED2_PWM_TRIS
#define LED2_LAT                      (LATAbits.LATA2)
#define LED2_PORT                     (PORTAbits.RA2)
#define LED2_TSEL                     (PRODL)

// LED3 is pin 7
#define LED3_USE_PWM 3
#define LED3_ANSEL                    (ANSELAbits.ANSA5)
#define LED3_PWM_TRIS                 (TRISAbits.TRISA5)
#define LED3_TRIS LED3_PWM_TRIS
#define LED3_LAT                      (LATAbits.LATA5)
#define LED3_PORT                     (PORTAbits.RA5)
#define LED3_TSEL                     (PRODL)

// LED4 is connected to PIN 14
#define LED4_USE_PWM 3
#define LED4_ANSEL                    (ANSELCbits.ANSC3)
#define LED4_PWM_TRIS                 (TRISCbits.TRISC3)
#define LED4_TRIS LED4_PWM_TRIS
#define LED4_LAT                      (LATCbits.LATC3)
#define LED4_PORT                     (PORTCbits.RC3)
#define LED4_TSEL                     (PRODL)

// LED5 is connected to PIN 25
#define LED5_USE_PWM 3
#define LED5_ANSEL                    (ANSELBbits.ANSB4)
#define LED5_PWM_TRIS                 (TRISBbits.TRISB4)
#define LED5_TRIS LED5_PWM_TRIS
#define LED5_LAT                      (LATBbits.LATB4)
#define LED5_PORT                     (PORTBbits.RB4)
#define LED5_TSEL                     (PRODL)

// LED6 is connected to PIN 24
#define LED6_USE_PWM 3
#define LED6_ANSEL                    (ANSELBbits.ANSB3)
#define LED6_PWM_TRIS                 (TRISBbits.TRISB3)
#define LED6_TRIS LED6_PWM_TRIS
#define LED6_LAT                      (LATBbits.LATB3)
#define LED6_PORT                     (PORTBbits.RB3)
#define LED6_TSEL                     (PRODL)
	// CCP1
//#pragma config CCP2MX=0
// the above is default

// LED7 is connected to pin 23
#define LED7_USE_PWM 3
#define LED7_ANSEL                    (ANSELBbits.ANSB2)
#define LED7_PWM_TRIS                 (TRISBbits.TRISB2)
#define LED7_TRIS LED7_PWM_TRIS
#define LED7_LAT                      (LATBbits.LATB2)
#define LED7_PORT                     (PORTBbits.RB2)
#define LED7_TSEL                     (PRODL)

// LED8 is connected to PIN 22
#define LED8_USE_PWM 3
#define LED8_ANSEL                    (ANSELBbits.ANSB1)
#define LED8_PWM_TRIS                 (TRISBbits.TRISB1)
#define LED8_TRIS LED8_PWM_TRIS
#define LED8_LAT                      (LATBbits.LATB1)
#define LED8_PORT                     (PORTBbits.RB1)
#define LED8_TSEL                     (PRODL)

// Additional LED outputs
// LED9 pin pin 21
#define LED9_USE_PWM 3
#define LED9_ANSEL                    (ANSELBbits.ANSB0)
#define LED9_TRIS                     (TRISBbits.TRISB0)
#define LED9_LAT                      (LATBbits.LATB0)

// LED10 pin 12
#define LED10_USE_PWM 3
#define LED10_ANSEL                    (PRODL)
#define LED10_LAT                      (LATCbits.LATC1)
#define LED10_TRIS                     (TRISCbits.TRISC1)

// LED11 pin 13
#define LED11_USE_PWM 3
#define LED11_ANSEL                    (ANSELCbits.ANSC2)
#define LED11_LAT                      (LATCbits.LATC2)
#define LED11_TRIS                     (TRISCbits.TRISC2)

// LED12 pin 6
#define LED12_USE_PWM 3
#define LED12_ANSEL                   (PRODL)
#define LED12_LAT                     (LATAbits.LATA4)
#define LED12_TRIS                    (TRISAbits.TRISA4)
// LED13 pin 26
#define LED13_USE_PWM 3
#define LED13_ANSEL                   (ANSELBbits.ANSB5)
#define LED13_LAT                     (LATBbits.LATB5)
#define LED13_TRIS                    (TRISBbits.TRISB5)

// LED14 pin 9
#define LED14_USE_PWM 3
#define LED14_ANSEL                   (PRODL)
#define LED14_LAT                     (LATAbits.LATA7)
#define LED14_TRIS                    (TRISAbits.TRISA7)

// LED15 pin 5
#define LED15_USE_PWM 3
#define LED15_ANSEL                   (ANSELAbits.ANSA3)
#define LED15_LAT                     (LATAbits.LATA3)
#define LED15_TRIS                    (TRISAbits.TRISA3)

// LED16 pin 3
#define LED16_USE_PWM 3
#define LED16_ANSEL                   (ANSELAbits.ANSA1)
#define LED16_LAT                     (LATAbits.LATA1)
#define LED16_TRIS                    (TRISAbits.TRISA1)

#define MIN_MANUAL_PWM 0
#define MAX_MANUAL_PWM 0

// TODO define analog input pins for light controller configuration


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
