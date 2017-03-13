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

#ifndef LIGHT_CONTROLLER
// SW2 is connected to PIN 21
#define SW2_ANSEL                    (ANSELBbits.ANSB1)
#define SW2_CHS                      10
#define SW2_TRIS                     (TRISBbits.TRISB1)
#define SW2_LAT                      (LATBbits.LATB1)
#define SW2_PORT                     (PORTBbits.RB1)
#define SW2_WPUB                     (WPUBbits.WPUB1)
// SW1 is connected to PIN 22
#define SW1_ANSEL                    (ANSELBbits.ANSB2)
#define SW1_CHS                      8
#define SW1_TRIS                     (TRISBbits.TRISB2)
#define SW1_LAT                      (LATBbits.LATB2)
#define SW1_PORT                     (PORTBbits.RB2)
#define SW1_WPUB                     (WPUBbits.WPUB2)

// SW3 is connected to PIN 23
#define SW4_ANSEL                    (ANSELBbits.ANSB3)
#define SW4_CHS                      9
#define SW4_TRIS                     (TRISBbits.TRISB3)
#define SW4_LAT                      (LATBbits.LATB3)
#define SW4_PORT                     (PORTBbits.RB3)
#define SW4_WPUB                     (WPUBbits.WPUB3)

// SW4 is connected to PIN 24
#define SW3_ANSEL                    (ANSELBbits.ANSB4)
#define SW3_CHS                      11
#define SW3_TRIS                     (TRISBbits.TRISB4)
#define SW3_LAT                      (LATBbits.LATB4)
#define SW3_PORT                     (PORTBbits.RB4)
#define SW3_WPUB                     (WPUBbits.WPUB4)
#endif

#if defined(LIGHT_CONTROLLER) || defined(AMPLIFIER)
#define SWITCH_COUNT 0
#else
#define SWITCH_COUNT 4
#endif

#if !defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
// LED1 is connected to PIN 12
#define LED1_ANSEL                    (PRODL)
#define LED1_TRIS                     (TRISCbits.TRISC1)
#define LED1_LAT                      (LATCbits.LATC1)
#define LED1_PORT                     (PORTCbits.RC1)
	// CCP2
#define LED1_TSEL                     (CCPTMRS0bits.C2TSEL)
#define LED1_PWM_TRIS                 (LED1_TRIS)
#define LED1_CCPM                     (CCP2CONbits.CCP2M)
#define LED1_DCB                      (CCP2CONbits.DC2B)
#define LED1_CPPRL                    (CCPR2L)
#define LED1_PWMMODE                  (CCP2CONbits.P2M)
#define LED1_REVERSE_LEVEL 1
#define LED1_USE_PWM 1
// LED2 is connected to PIN 21
#define LED2_ANSEL                    (ANSELBbits.ANSB0)
#define LED2_TRIS                     (TRISBbits.TRISB0)
#define LED2_LAT                      (LATBbits.LATB0)
#define LED2_PORT                     (PORTBbits.RB0)
#define LED2_TSEL                     (CCPTMRS1bits.C4TSEL)
#define LED2_PWM_TRIS                 (LED2_TRIS)
#define LED2_CCPM                     (CCP4CONbits.CCP4M)
#define LED2_DCB                      (CCP4CONbits.DC4B)
#define LED2_CPPRL                    (CCPR4L)
#define LED2_PWMMODE                  (PRODL)
#define LED2_REVERSE_LEVEL 1
#define LED2_USE_PWM 1
// LED3 is connected to PIN 26
#define LED3_ANSEL                    (ANSELBbits.ANSB5)
#define LED3_TRIS                     (TRISBbits.TRISB5)
#define LED3_LAT                      (LATBbits.LATB5)
#define LED3_PORT                     (PORTBbits.RB5)
	// P1D should have used 26 which is CCP3
#define LED3_TSEL                     (CCPTMRS0bits.C3TSEL)
#define LED3_PWM_TRIS                 (LED3_TRIS)
#define LED3_CCPM                     (CCP3CONbits.CCP3M)
#define LED3_DCB                      (CCP3CONbits.DC3B)
#define LED3_CPPRL                    (CCPR3L)
#define LED3_PWMMODE                  (CCP3CONbits.P3M)
#define LED3_REVERSE_LEVEL 1
#define LED3_USE_PWM 1	
// LED4 is connected to PIN 13
#define LED4_ANSEL                    (ANSELCbits.ANSC2)
#define LED4_TRIS                     (TRISCbits.TRISC2)
#define LED4_LAT                      (LATCbits.LATC2)
#define LED4_PORT                     (PORTCbits.RC2)
	// CCP1
//#pragma config CCP2MX=0
// the above is default
#define LED4_TSEL                     (CCPTMRS0bits.C1TSEL)
#define LED4_PWM_TRIS                 (LED4_TRIS)
#define LED4_CCPM                     (CCP1CONbits.CCP1M)
#define LED4_DCB                      (CCP1CONbits.DC1B)
#define LED4_CPPRL                    (CCPR1L)
#define LED4_PWMMODE                  (CCP1CONbits.P1M)
#define LED4_REVERSE_LEVEL 1
#define LED4_USE_PWM 1

#define NO_MANUAL_PWM 1
#define MIN_MANUAL_PWM 4
#endif

#if defined(AMPLIFIER)
#define LED_COUNT 16
#elif defined(LIGHT_CONTROLLER)
#define LED_COUNT 8
#else
#define LED_COUNT 4
#endif

#if defined(LIGHT_CONTROLLER)
#if !defined(AMPLIFIER)
// Additional ADC channels
// ADC1 pin 2
#define ADC1_ANSEL                    (ANSELAbits.ANSA0)
#define ADC1_CHS                      0
#define ADC1_TRIS                     (TRISAbits.TRISA0)

// ADC2 pin 3
#define ADC2_ANSEL                    (ANSELAbits.ANSA2)
#define ADC2_CHS                      2
#define ADC2_TRIS                     (TRISAbits.TRISA2)

// ADC3 pin 4
#define ADC3_ANSEL                    (ANSELAbits.ANSA5)
#define ADC3_CHS                      4
#define ADC3_TRIS                     (TRISAbits.TRISA5)

// ADC4 pin 14
#define ADC4_ANSEL                   (ANSELCbits.ANSC3)
#define ADC4_CHS                     15
#define ADC4_TRIS                    (TRISCbits.TRISC3)
// ADC5 pin 25
#define ADC5_ANSEL                   (ANSELBbits.ANSB4)
#define ADC5_CHS                     11
#define ADC5_TRIS                    (TRISBbits.TRISB4)

// ADC6 pin 24
#define ADC6_ANSEL                   (ANSELBbits.ANSB3)
#define ADC6_CHS                     9
#define ADC6_TRIS                    (TRISBbits.TRISB3)

// ADC7 pin 23
#define ADC7_ANSEL                   (ANSELBbits.ANSB2)
#define ADC7_CHS                     8
#define ADC7_TRIS                    (TRISBbits.TRISB2)

// ADC8 pin 22
#define ADC8_ANSEL                   (ANSELBbits.ANSB1)
#define ADC8_CHS                     10
#define ADC8_TRIS                    (TRISBbits.TRISB1)

// ADC9 pin 16
#define ADC9_ANSEL                   (ANSELCbits.ANSC5)
#define ADC9_CHS                     17
#define ADC9_TRIS                    (TRISCbits.TRISC5)

#else
// Additional LED outputs
// ADC1 pin 2
#define LED9_ANSEL                    (ANSELAbits.ANSA0)
#define LED9_TRIS                     (TRISAbits.TRISA0)
#define LED9_LAT                      (LATAbits.LATA0)

// ADC2 pin 3
#define LED10_ANSEL                    (ANSELAbits.ANSA2)
#define LED10_LAT                      (LATAbits.LATA2)
#define LED10_TRIS                     (TRISAbits.TRISA2)

// ADC3 pin 4
#define LED11_ANSEL                    (ANSELAbits.ANSA5)
#define LED11_LAT                      (LATAbits.LATA5)
#define LED11_TRIS                     (TRISAbits.TRISA5)

// ADC4 pin 14
#define LED12_ANSEL                   (ANSELCbits.ANSC3)
#define LED12_LAT                     (LATCbits.LATC3)
#define LED12_TRIS                    (TRISCbits.TRISC3)
// ADC5 pin 25
#define LED13_ANSEL                   (ANSELBbits.ANSB4)
#define LED13_LAT                     (LATBbits.LATB4)
#define LED13_TRIS                    (TRISBbits.TRISB4)

// ADC6 pin 24
#define LED14_ANSEL                   (ANSELBbits.ANSB3)
#define LED14_LAT                     (LATBbits.LATB3)
#define LED14_TRIS                    (TRISBbits.TRISB3)

// ADC7 pin 23
#define LED15_ANSEL                   (ANSELBbits.ANSB2)
#define LED15_LAT                     (LATBbits.LATB2)
#define LED15_TRIS                    (TRISBbits.TRISB2)

// ADC8 pin 22
#define LED16_ANSEL                   (ANSELBbits.ANSB1)
#define LED16_LAT                     (LATBbits.LATB1)
#define LED16_TRIS                    (TRISBbits.TRISB1)
#endif
// LED1 is pin 3
#define LED1_USE_PWM 0
#define LED1_ANSEL                    (PRODL)
#define LED1_PWM_TRIS                 (TRISAbits.TRISA1)
#define LED1_TRIS LED1_PWM_TRIS
#define LED1_LAT                      (LATAbits.LATA1)
#define LED1_PORT                     (PORTAbits.RA1)
#define LED1_TSEL                     (PRODL)
// LED2 is pin 5
#define LED2_USE_PWM 0
#define LED2_ANSEL                    (PRODL)
#define LED2_PWM_TRIS                 (TRISAbits.TRISA3)
#define LED2_TRIS LED2_PWM_TRIS
#define LED2_LAT                      (LATAbits.LATA3)
#define LED2_PORT                     (PORTAbits.RA3)
#define LED2_TSEL                     (PRODL)

// LED3 is pin 9
#define LED3_USE_PWM 0
#define LED3_ANSEL                    (PRODL)
#define LED3_PWM_TRIS                 (TRISAbits.TRISA7)
#define LED3_TRIS LED3_PWM_TRIS
#define LED3_LAT                      (LATAbits.LATA7)
#define LED3_PORT                     (PORTAbits.RA7)
#define LED3_TSEL                     (PRODL)

// LED4 is connected to PIN 26
#define LED4_ANSEL                    (ANSELBbits.ANSB5)
#define LED4_TRIS                     (TRISBbits.TRISB5)
#define LED4_LAT                      (LATBbits.LATB5)
#define LED4_PORT                     (PORTBbits.RB5)
	// CCP3 is on pin 26
#define LED4_TSEL                     (CCPTMRS0bits.C3TSEL)
#define LED4_PWM_TRIS                 (LED4_TRIS)
#define LED4_CCPM                     (CCP3CONbits.CCP3M)
#define LED4_DCB                      (CCP3CONbits.DC3B)
#define LED4_CPPRL                    (CCPR3L)
#define LED4_PWMMODE                  (PRODL)
#define LED4_REVERSE_LEVEL 1
#define LED4_USE_PWM 1

// LED5 is connected to PIN 6
#define LED5_ANSEL                    (PRODL)
#define LED5_TRIS                     (TRISAbits.TRISA4)
#define LED5_LAT                      (LATAbits.LATA4)
#define LED5_PORT                     (PORTAbits.RA4)
#define LED5_TSEL                     (CCPTMRS1bits.C5TSEL)
#define LED5_PWM_TRIS                 (LED5_TRIS)
#define LED5_CCPM                     (CCP5CONbits.CCP5M)
#define LED5_DCB                      (CCP5CONbits.DC5B)
#define LED5_CPPRL                    (CCPR5L)
#define LED5_PWMMODE                  (PRODL)
#define LED5_REVERSE_LEVEL 1
#define LED5_USE_PWM 1	

// LED6 is connected to PIN 13
#define LED6_ANSEL                    (ANSELCbits.ANSC2)
#define LED6_TRIS                     (TRISCbits.TRISC2)
#define LED6_LAT                      (LATCbits.LATC2)
#define LED6_PORT                     (PORTCbits.RC2)
	// CCP1
//#pragma config CCP2MX=0
// the above is default
#define LED6_TSEL                     (CCPTMRS0bits.C1TSEL)
#define LED6_PWM_TRIS                 (LED6_TRIS)
#define LED6_CCPM                     (CCP1CONbits.CCP1M)
#define LED6_DCB                      (CCP1CONbits.DC1B)
#define LED6_CPPRL                    (CCPR1L)
#define LED6_PWMMODE                  (CCP1CONbits.P1M)
#define LED6_REVERSE_LEVEL 1
#define LED6_USE_PWM 1

// LED7 is connected to pin 12
#define LED7_ANSEL                    (PRODL)
#define LED7_TRIS                     (TRISCbits.TRISC1)
#define LED7_LAT                      (LATCbits.LATC1)
#define LED7_PORT                     (PORTCbits.RC1)
	// CCP2
#define LED7_TSEL                     (CCPTMRS0bits.C2TSEL)
#define LED7_PWM_TRIS                 (LED7_TRIS)
#define LED7_CCPM                     (CCP2CONbits.CCP2M)
#define LED7_DCB                      (CCP2CONbits.DC2B)
#define LED7_CPPRL                    (CCPR2L)
#define LED7_PWMMODE                  (CCP2CONbits.P2M)
#define LED7_REVERSE_LEVEL 1
#define LED7_USE_PWM 1

// LED2 is connected to PIN 21
#define LED8_ANSEL                    (ANSELBbits.ANSB0)
#define LED8_TRIS                     (TRISBbits.TRISB0)
#define LED8_LAT                      (LATBbits.LATB0)
#define LED8_PORT                     (PORTBbits.RB0)
#define LED8_TSEL                     (CCPTMRS1bits.C4TSEL)
#define LED8_PWM_TRIS                 (LED8_TRIS)
#define LED8_CCPM                     (CCP4CONbits.CCP4M)
#define LED8_DCB                      (CCP4CONbits.DC4B)
#define LED8_CPPRL                    (CCPR4L)
#define LED8_PWMMODE                  (PRODL)
#define LED8_REVERSE_LEVEL 1
#define LED8_USE_PWM 1

#define MIN_MANUAL_PWM 0
#define MAX_MANUAL_PWM 3
#endif

#ifdef AMPLIFIER
#if 0
// 8 pairs of MUTE and STANDBY
// AMP1 - Mute = RB1, Standby = RB2 - bottom left
#define AMP1_MUTE_ANSEL             (ANSELBbits.ANSB1)
#define AMP1_MUTE_TRIS              (TRISBbits.TRISB1)
#define AMP1_MUTE_LAT               (LATBbits.LATB1)
#define AMP1_MUTE_PORT              (PORTBbits.RB1)
#define AMP1_STBY_ANSEL             (ANSELBbits.ANSB2)
#define AMP1_STBY_TRIS              (TRISBbits.TRISB2)
#define AMP1_STBY_LAT               (LATBbits.LATB2)
#define AMP1_STBY_PORT              (PORTBbits.RB2)
// AMP2 - Mute = RB3 (24), Standby = RB4 (25)
#define AMP2_MUTE_ANSEL             (ANSELBbits.ANSB3)
#define AMP2_MUTE_TRIS              (TRISBbits.TRISB3)
#define AMP2_MUTE_LAT               (LATBbits.LATB3)
#define AMP2_MUTE_PORT              (PORTBbits.RB3)
#define AMP2_STBY_ANSEL             (ANSELBbits.ANSB4)
#define AMP2_STBY_TRIS              (TRISBbits.TRISB4)
#define AMP2_STBY_LAT               (LATBbits.LATB4)
#define AMP2_STBY_PORT              (PORTBbits.RB4)
// AMP3 - Mute = RC3 (14), Standby = RA5 (7)
#define AMP3_MUTE_ANSEL             (ANSELCbits.ANSC3)
#define AMP3_MUTE_TRIS              (TRISCbits.TRISC3)
#define AMP3_MUTE_LAT               (LATCbits.LATC3)
#define AMP3_MUTE_PORT              (PORTCbits.RC3)
#define AMP3_STBY_ANSEL             (ANSELAbits.ANSA5)
#define AMP3_STBY_TRIS              (TRISAbits.TRISA5)
#define AMP3_STBY_LAT               (LATAbits.LATA5)
#define AMP3_STBY_PORT              (PORTAbits.RA5)
// AMP4 - Mute = RA2  (4), Standby = RAO (2)
#define AMP4_MUTE_ANSEL             (ANSELAbits.ANSA2)
#define AMP4_MUTE_TRIS              (TRISAbits.TRISA2)
#define AMP4_MUTE_LAT               (LATAbits.LATA2)
#define AMP4_MUTE_PORT              (PORTAbits.RA2)
#define AMP4_STBY_ANSEL             (ANSELAbits.ANSA0)
#define AMP4_STBY_TRIS              (TRISAbits.TRISA0)
#define AMP4_STBY_LAT               (LATAbits.LATA0)
#define AMP4_STBY_PORT              (PORTAbits.RA0)
// AMP5 - Mute = RB0 (21), Standby = RC1 (12) - bottom right
#define AMP5_MUTE_ANSEL             (ANSELBbits.ANSB0)
#define AMP5_MUTE_TRIS              (TRISBbits.TRISB0)
#define AMP5_MUTE_LAT               (LATBbits.LATB0)
#define AMP5_MUTE_PORT              (PORTBbits.RB0)
#define AMP5_STBY_ANSEL             (PRODL)
#define AMP5_STBY_TRIS              (TRISCbits.TRISC1)
#define AMP5_STBY_LAT               (LATCbits.LATC1)
#define AMP5_STBY_PORT              (PORTCbits.RC1)
// AMP6 - Mute = RC2 (13), Standby = RA4 (6)
#define AMP6_MUTE_ANSEL             (ANSELCbits.ANSC2)
#define AMP6_MUTE_TRIS              (TRISCbits.TRISC2)
#define AMP6_MUTE_LAT               (LATCbits.LATC2)
#define AMP6_MUTE_PORT              (PORTCbits.RC2)
#define AMP6_STBY_ANSEL             (PRODL)
#define AMP6_STBY_TRIS              (TRISAbits.TRISA4)
#define AMP6_STBY_LAT               (LATAbits.LATA4)
#define AMP6_STBY_PORT              (PORTAbits.RA4)
// AMP7 - Mute = RB5 (26), Standby = RA7 (9)
#define AMP7_MUTE_ANSEL             (ANSELBbits.ANSB5)
#define AMP7_MUTE_TRIS              (TRISBbits.TRISB5)
#define AMP7_MUTE_LAT               (LATBbits.LATB5)
#define AMP7_MUTE_PORT              (PORTBbits.RB5)
#define AMP7_STBY_ANSEL             (PRODL)
#define AMP7_STBY_TRIS              (TRISAbits.TRISA7)
#define AMP7_STBY_LAT               (LATAbits.LATA7)
#define AMP7_STBY_PORT              (PORTAbits.RA7)
// AMP8 - Mute = RA3  (5), Standby = RA1 (3)
#define AMP8_MUTE_ANSEL             (ANSELAbits.ANSA3)
#define AMP8_MUTE_TRIS              (TRISAbits.TRISA3)
#define AMP8_MUTE_LAT               (LATAbits.LATA3)
#define AMP8_MUTE_PORT              (PORTAbits.RA3)
#define AMP8_STBY_ANSEL             (ANSELAbits.ANSA1)
#define AMP8_STBY_TRIS              (TRISAbits.TRISA1)
#define AMP8_STBY_LAT               (LATAbits.LATA1)
#define AMP8_STBY_PORT              (PORTAbits.RA1)
#define MIN_MANUAL_PWM 0
#define MAX_MANUAL_PWM 0
#endif
#endif
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
