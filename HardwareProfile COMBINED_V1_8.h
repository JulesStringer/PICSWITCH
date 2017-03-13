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

// SW1 is connected to PIN 26
#define SW1_ANSEL                    (ANSELBbits.ANSB5)
#define SW1_CHS                      10
#define SW1_TRIS                     (TRISBbits.TRISB5)
#define SW1_LAT                      (LATBbits.LATB5)
#define SW1_PORT                     (PORTBbits.RB5)
#define SW1_WPUB                     (WPUBbits.WPUB5)

#define SWITCH_COUNT 1

// LED1 is connected to PIN 21
#define LED1_ANSEL                    (ANSELBbits.ANSB0)
#define LED1_TRIS                     (TRISBbits.TRISB0)
#define LED1_LAT                      (LATBbits.LATB0)
#define LED1_PORT                     (PORTBbits.RB0)
#define LED1_TSEL                     (CCPTMRS1bits.C4TSEL)
#define LED1_PWM_TRIS                 (LED1_TRIS)
#define LED1_CCPM                     (CCP4CONbits.CCP4M)
#define LED1_DCB                      (CCP4CONbits.DC4B)
#define LED1_CPPRL                    (CCPR4L)
#define LED1_PWMMODE                  (PRODL)
#define LED1_REVERSE_LEVEL 1
#define LED1_USE_PWM 1

// LED2 is connected to PIN 12
#define LED2_ANSEL                    (PRODL)
#define LED2_TRIS                     (TRISCbits.TRISC1)
#define LED2_LAT                      (LATCbits.LATC1)
#define LED2_PORT                     (PORTCbits.RC1)
	// CCP2
#define LED2_TSEL                     (CCPTMRS0bits.C2TSEL)
#define LED2_PWM_TRIS                 (LED2_TRIS)
#define LED2_CCPM                     (CCP2CONbits.CCP2M)
#define LED2_DCB                      (CCP2CONbits.DC2B)
#define LED2_CPPRL                    (CCPR2L)
#define LED2_PWMMODE                  (CCP2CONbits.P2M)
#define LED2_REVERSE_LEVEL 1
#define LED2_USE_PWM 1

// LED3 is connected to PIN 13
#define LED3_ANSEL                    (ANSELCbits.ANSC2)
#define LED3_TRIS                     (TRISCbits.TRISC2)
#define LED3_LAT                      (LATCbits.LATC2)
#define LED3_PORT                     (PORTCbits.RC2)
	// CCP1
//#pragma config CCP2MX=0
// the above is default
#define LED3_TSEL                     (CCPTMRS0bits.C1TSEL)
#define LED3_PWM_TRIS                 (LED3_TRIS)
#define LED3_CCPM                     (CCP1CONbits.CCP1M)
#define LED3_DCB                      (CCP1CONbits.DC1B)
#define LED3_CPPRL                    (CCPR1L)
#define LED3_PWMMODE                  (CCP1CONbits.P1M)
#define LED3_REVERSE_LEVEL 1
#define LED3_USE_PWM 1
// LED4 is connected to PIN 6
#define LED4_ANSEL                    (PRODL)
#define LED4_TRIS                     (TRISAbits.TRISA4)
#define LED4_LAT                      (LATAbits.LATA4)
#define LED4_PORT                     (PORTAbits.RA4)
#define LED4_TSEL                     (CCPTMRS1bits.C5TSEL)
#define LED4_PWM_TRIS                 (LED4_TRIS)
#define LED4_CCPM                     (CCP5CONbits.CCP5M)
#define LED4_DCB                      (CCP5CONbits.DC5B)
#define LED4_CPPRL                    (CCPR5L)
#define LED4_PWMMODE                  (PRODL)
#define LED4_REVERSE_LEVEL 1
#define LED4_USE_PWM 1	

#define NO_MANUAL_PWM 1
#define MIN_MANUAL_PWM 5
#define LED_COUNT 4

// Additional ADC channels
// ADC1 pin 22
#define ADC1_ANSEL                    (ANSELBbits.ANSB1)
#define ADC1_CHS                      10
#define ADC1_TRIS                     (TRISBbits.TRISB1)

// ADC2 pin 23
#define ADC2_ANSEL                    (ANSELBbits.ANSB2)
#define ADC2_CHS                      8
#define ADC2_TRIS                     (TRISBbits.TRISB2)

// ADC3 pin 24
#define ADC3_ANSEL                    (ANSELBbits.ANSB3)
#define ADC3_CHS                      9
#define ADC3_TRIS                     (TRISBbits.TRISB3)

// ADC4 pin 25
#define ADC4_ANSEL                   (ANSELBbits.ANSB4)
#define ADC4_CHS                     11
#define ADC4_TRIS                    (TRISBbits.TRISB4)

// ADC9 pin 16

#define ADC9_ANSEL                   (ANSELCbits.ANSC5)
#define ADC9_CHS                     17
#define ADC9_TRIS                    (TRISCbits.TRISC5)


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
