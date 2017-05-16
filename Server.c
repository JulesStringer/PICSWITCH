/*=============================================================
Light Switch

Generates lighting commands and sends these to controller.

*/
#include <xc.h>
#include <string.h>

#include "HardwareProfile.h"

#include "C:\DEV\HomeAutomation\include\voltages.h"
//#include "C:\DEV\HomeAutomation\include\reasons.h"

// Should be incremented when each change is rolled out.
#define SOFTWARE_VERSION 1

#ifndef RS485_DEVICE
#ifndef INPUT_UART2
#define ECHO_UART1
#else
#if LED_OFFSET == 0
#define ECHO_UART2
#endif
#endif
#endif

void InitializeBoard(void);

BYTE g_TransmitToken = MASTER_DEVICE;
BYTE g_SlaveDevice = 0;
unsigned long g_tTransmitTokenValidTick = 0;
#define TICKS_TOKEN_VALID (TICKS_PER_SECOND / 10)
// TICKS_TOKEN_VALID assumes:
// 19200 Baud so 1745 characters per second.
// giving 0.573 milliseconds per character.
// Worst case is status message with battery processing,
// which has 104 characters.
// This will take 0.0596 seconds to transmit.
// To allow margin for error allow 0.1 seconds to transmit.
volatile BYTE UARTtimerExtension1 = 0;
volatile BYTE UARTtimerExtension2 = 0;

#define RCBUFSIZE 128

BYTE PopBuffer(volatile BYTE* pszBuffer, volatile BYTE* pnHead, volatile BYTE* pnTail)
{
    BYTE ch = 0;
    if ( *pnHead != *pnTail )
    {
        ch = pszBuffer[*pnHead];
        (*pnHead)++;
        if ( *pnHead >= RCBUFSIZE)
        {
            *pnHead = 0;
        }
    }
    return ch;
}
#if 0
void PushBuffer(volatile BYTE* pszBuffer, volatile BYTE* pnTail, BYTE ch)
{
    if ( ch != 0 )
    {
        pszBuffer[*pnTail] = ch;
        (*pnTail)++;
        if ( *pnTail >= RCBUFSIZE )
        {
            *pnTail = 0;
        }
    }
}
#endif
BYTE TestBuffer(BYTE nHead, BYTE nTail)
{
    return nHead != nTail;
}
void InitBuffer(volatile BYTE* pnHead, volatile BYTE* pnTail)
{
    *pnHead = 0;
    *pnTail = 0;
}
volatile BYTE szRCBUF[RCBUFSIZE];
volatile BYTE rcHead=0;
volatile BYTE rcTail=0;

volatile BYTE szRCBUF2[RCBUFSIZE];
volatile BYTE rcHead2=0;
volatile BYTE rcTail2=0;

#ifdef INPUT_SPI
volatile BYTE szRCBUF3[RCBUFSIZE];
volatile BYTE rcHead3 = 0;
volatile BYTE rcTail3 = 0;
volatile BYTE szTXBUF[RCBUFSIZE];
volatile BYTE txHead = 0;
volatile BYTE txTail = 0;
#endif

BYTE g_LEDLevels[LED_COUNT];
BYTE g_dutyCycle = 0;
BYTE g_resetDutyCycle = 0;
BYTE g_curLED = 0;
BYTE g_curLevel = 0;
BYTE g_byState = 0;
BYTE g_bySlaveState = 0;
BYTE g_byCommand = 0;
#if !defined(AMPLIFIER) && !defined(ANALOG_AUDIO)
#define MANUAL_PWM_USE_TIMER 0
#endif
//
// PIC18 Interrupt Service Routines
// 
// NOTE: Several PICs, including the PIC18F4620 revision A3 have a RETFIE FAST/MOVFF bug
// The interruptlow keyword is used to work around the bug when using C18
void DoManualPWM(void);

void interrupt high_priority HighISR(void)
{
//    static BYTE ch;
    if ( INTCONbits.TMR0IF )
    {
        INTCONbits.TMR0IF = 0;
        UARTtimerExtension1++;
        if ( UARTtimerExtension1 == 0 )
        {
            UARTtimerExtension2++;
        }
    }
#ifdef INPUT_SPI
    if ( PIR1bits.SSP1IF )
    {
        PIR1bits.SSP1IF = 0;
        if ( SSP1CON1bits.SSPOV)
        {
            SSP1CON1bits.SSPEN = 0;
            SSP1CON1bits.SSPEN = 1;
        }
        BYTE ch = SSP1BUF;
//        TXSTA1bits.TXEN = 1;
        if ( ch )
        {
            szRCBUF3[rcTail3] = ch;
            rcTail3++;
            rcTail3 &= 0x7F;
//            TXREG1 = ch;
        }
        if ( txHead != txTail )
        {
            SSP1BUF = szTXBUF[txHead];
            txHead++;
            txHead &= 0x7F;
        }
        else
        {
            SSP1BUF = 0;
        }
        SSP1STATbits.BF = 0;
        SSP1CON1bits.WCOL = 0;
    }
#endif
#ifndef INPUT_UART2
    if ( PIR1bits.RC1IF )
    {
        BYTE byOverflow = RCSTA1bits.OERR;
        szRCBUF[rcTail] = RCREG1;
        if ( byOverflow )
        {
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1; 
        }
        PIR1bits.RC1IF = 0;
#ifdef RS485_DEVICE
        // Only interested in input from master device if rs485
        if ( g_TransmitToken == MASTER_DEVICE && szRCBUF[rcTail])
#endif
        {  
            rcTail++;
            rcTail &= 0x7F;
        }
    }
#endif
#if defined(INPUT_UART2) || defined(FORWARD_UART2)
    if (PIR3bits.RC2IF )
    {
        BYTE byOverflow = RCSTA2bits.OERR;
        szRCBUF2[rcTail2] = RCREG2;
        if ( byOverflow )
        {
            RCSTA2bits.CREN=0;
            RCSTA2bits.CREN=1;
        }
        PIR3bits.RC2IF=0;
        rcTail2++;
        rcTail2 &= 0x7F;
    }
#endif
#ifdef MANUAL_PWM_USE_TIMER
    if ( PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;
        g_dutyCycle++;
        BYTE* pLEDLevel = g_LEDLevels + MIN_MANUAL_PWM;
        if ( g_dutyCycle == 0)
        {
#if LED1_USE_PWM == 0
            LED1_LAT = *pLEDLevel ? 1 : 0;
#endif
#if MIN_MANUAL_PWM == 0 && MAX_MANUAL_PWM >= 1
            pLEDLevel++;
#endif
#if LED2_USE_PWM == 0
            LED2_LAT = *pLEDLevel ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 2 && MAX_MANUAL_PWM >= 2
            pLEDLevel++;
#endif
#if LED3_USE_PWM == 0
            LED3_LAT = *pLEDLevel ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 3 && MAX_MANUAL_PWM >= 3
            pLEDLevel++;
#endif
#if LED4_USE_PWM == 0
            LED4_LAT = *pLEDLevel ? 1 : 0;
#endif
            
#if MIN_MANUAL_PWM < 4 && MAX_MANUAL_PWM >= 4
            pLEDLevel++;
#endif
#if LED_COUNT > 4
#if LED5_USE_PWM == 0
            LED5_LAT = *pLEDLevel ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 5 && MAX_MANUAL_PWM >= 5
            pLEDLevel++;
#endif
#if LED6_USE_PWM == 0
            LED6_LAT = *pLEDLevel ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 6 && MAX_MANUAL_PWM >= 6
            pLEDLevel++;
#endif
#if LED7_USE_PWM == 0
            LED7_LAT = *pLEDLevel ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 7 && MAX_MANUAL_PWM >= 7
            pLEDLevel++;
#endif
#if LED8_USE_PWM == 0
            LED8_LAT = *pLEDLevel ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 8 && MAX_MANUAL_PWM >= 8
            pLEDLevel++;
#endif
#endif
//            g_resetDutyCycle = 1;
        }
        else
        {
#if LED1_USE_PWM == 0
            LED1_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM == 0 && MAX_MANUAL_PWM >= 1
            pLEDLevel++;
#endif
#if LED2_USE_PWM == 0
            LED2_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 2 && MAX_MANUAL_PWM >= 2
            pLEDLevel++;
#endif
#if LED3_USE_PWM == 0
            LED3_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 3 && MAX_MANUAL_PWM >= 3
            pLEDLevel++;
#endif
#if LED4_USE_PWM == 0
            LED4_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 4 && MAX_MANUAL_PWM >= 4
            pLEDLevel++;
#endif
#if LED_COUNT > 4
#if LED5_USE_PWM == 0
            LED5_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 5 && MAX_MANUAL_PWM >= 5
            pLEDLevel++;
#endif
#if LED6_USE_PWM == 0
            LED6_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 6 && MAX_MANUAL_PWM >= 6
            pLEDLevel++;
#endif
#if LED7_USE_PWM == 0
            LED7_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif            
#if MIN_MANUAL_PWM < 7 && MAX_MANUAL_PWM >= 7
            pLEDLevel++;
#endif
#if LED8_USE_PWM == 0
            LED8_LAT = (*pLEDLevel >  g_dutyCycle) ? 1 : 0;
#endif
#if MIN_MANUAL_PWM < 8 && MAX_MANUAL_PWM >= 8
            pLEDLevel++;
#endif
#endif
        }
    }
#endif
}
BYTE GetChar1(void)
{
    return PopBuffer(szRCBUF, &rcHead, &rcTail);
    //return 0;
}
BYTE GetChar2(void)
{
    return PopBuffer(szRCBUF2, &rcHead2, &rcTail2);
}
#ifdef INPUT_SPI
BYTE GetChar3(void)
{
    return PopBuffer(szRCBUF3, &rcHead3, &rcTail3);
}
#endif
BYTE GetChar(void)
{
#ifdef INPUT_SPI
    return GetChar3();
#else
#ifdef INPUT_UART2
    return GetChar2();
#else
    return GetChar1();
#endif
#endif
}
int HaveChar(void)
{
#ifdef INPUT_SPI
    return TestBuffer(rcHead3, rcTail3);
#else
#ifdef INPUT_UART2
    return TestBuffer(rcHead2, rcTail2);
#else
    return TestBuffer(rcHead, rcTail);
#endif
#endif
}
unsigned long GetTickCount(void)
{
	typedef union _TICK
	{
	    DWORD Val;
	    struct _TICK_bytes
	    {
	        BYTE b0;
	        BYTE b1;
	        BYTE b2;
	        BYTE b3;
	    } byte;
	} _TICK;
    _TICK currentTime;

    BYTE IntFlag1;
    BYTE IntFlag2;

    /* copy the byte extension */
    currentTime.byte.b2 = 0;
    currentTime.byte.b3 = 0;

    INTCONbits.TMR0IE = 0;
    do
    {
        IntFlag1 = INTCONbits.TMR0IF;
        currentTime.byte.b0 = TMR0L;
        currentTime.byte.b1 = TMR0H;
        IntFlag2 = INTCONbits.TMR0IF;
    } while(IntFlag1 != IntFlag2);

    if( IntFlag1 > 0 )
    {
        INTCONbits.TMR0IF = 0;
        UARTtimerExtension1++;
        if(UARTtimerExtension1 == 0)
        {
            UARTtimerExtension2++;
        }
    }
    currentTime.byte.b2 += UARTtimerExtension1;
    currentTime.byte.b3 += UARTtimerExtension2;
    INTCONbits.TMR0IE = 1;
    return currentTime.Val;
}
volatile long Tick = 0;
#ifdef RS485_DEVICE
BYTE TransmitValid()
{
    if ( SWITCH_ID == g_TransmitToken )
    {
        return 1;
    }
    return 0;
}
#else
BYTE TransmitValid()
{
    return 1;
}
#endif
void SendBytes1(BYTE* pData, BYTE nLen)
{
    if ( TransmitValid() )
    {
        DI_IO = 1;
        TXSTAbits.TXEN=1;
        while(nLen > 0 )
        {
            TXREG1 = *pData;
            pData++;
            nLen--;
            while( !TXSTAbits.TRMT );
        }
        DI_IO = 0;
    }
}
void Delay(unsigned long ms)
{
    unsigned long tNext = GetTickCount() + TICKS_PER_SECOND * ms / 1000;
    // technically wrong at wrap around, but safe.
    if ( GetTickCount() > tNext )
    {
        while(GetTickCount() > 0);
    }
    while( GetTickCount() < tNext);
}
void SendBytes2(BYTE* pData, BYTE nLen)
{
    DI2_IO = 1;
    TXSTA2bits.TXEN=1;
    while(nLen > 0 )
    {
        TXREG2 = *pData;
        pData++;
        nLen--;
        while( !TXSTA2bits.TRMT );
#ifdef INPUT_UART2
        // Insert a small delay to ensure all characters go across
//        Delay(1);
#endif
    }
    DI2_IO = 0;
}
#ifdef INPUT_SPI
void SendBytes3(BYTE* pData, BYTE nLen)
{
    while(nLen > 0 )
    {
        PushBuffer(szTXBUF, &txTail, *pData);
        pData++;
        nLen--;
    }
}
#endif
void SendBytes(BYTE* pData, BYTE nLen)
{
#ifdef INPUT_SPI
    SendBytes3(pData, nLen);
#else
#ifdef INPUT_UART2
    SendBytes2(pData, nLen);
#else
    SendBytes1(pData, nLen);
#endif
#endif
}

void StartTimer(void)
{
    Tick = 0;
    T0CONbits.TMR0ON = 0; // disable timer
    INTCON2bits.TMR0IP = 1; // high priority interrupt
    T0CONbits.T08BIT = 0; // timer is 16 bit
    T0CONbits.T0CS = 0;   // Clock source is instruction cycle clock
    T0CONbits.T0SE = 0;   // Increment on low to high transition
    T0CONbits.PSA = 0;    // Using pre-scaler
    T0CONbits.T0PS = 7;   // use 256 times prescaler
    TMR0H = 0;
    TMR0L = 0;
    INTCONbits.TMR0IF = 0; // clear the interrupt flag
    INTCONbits.TMR0IE = 1; // enable the interrupt
    T0CONbits.TMR0ON = 1; // turn on timer
    INTCONbits.GIEL = 1;  // also need interrupts
}
#if defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
#ifdef PROCESS_BATTERY
static int g_nFITInterval = 2100;
static long g_TimeStartCharging = 0;
static long g_TimeStopCharging = 0;
static BYTE g_bCharging = 0;
static BYTE g_LastReason = 0;
static BYTE g_bChargingChange = 0;
static BYTE g_bManualCharging = 0;

void SetLED(BYTE byLED, BYTE byLevel);

void StopCharging(BYTE Reason)
{
    g_LastReason = Reason;
    g_bCharging = 0;
    g_bChargingChange = 1;
//    g_LEDLevels[CHARGER_PIN - 1] = 0;
    g_TimeStopCharging = GetTickCount();
    g_TimeStartCharging = 0;
    if ( g_LEDLevels[CHARGER_PIN - 1] != 0)
    {
        SetLED(CHARGER_PIN, 0);
    }
}
void StartCharging(BYTE Reason)
{
    g_LastReason = Reason;
    g_bCharging = 1;
    g_bChargingChange = 1;
//    g_LEDLevels[CHARGER_PIN - 1] = 255;
    g_TimeStartCharging = GetTickCount();
    g_TimeStopCharging = 0;
    if ( g_LEDLevels[CHARGER_PIN - 1] == 0 )
    {
        SetLED(CHARGER_PIN, 255);
    }
}
#endif
#define VOLTAGE_CHANNEL LED_COUNT
BYTE g_CurrentADC = 0;

typedef struct tagADCINFO
{
    BYTE m_Channel;
    BYTE m_Count;
    BYTE m_Discard;
}ADCINFO;
#define CURRENT_READINGS 20
#define CURRENT_DISCARD 4
#define VOLTAGE_READINGS 24
#define VOLTAGE_DISCARD 8
ADCINFO g_ChannelInfo[] =
{
    ADC1_CHS,CURRENT_READINGS + CURRENT_DISCARD,2*CURRENT_DISCARD // allow extra for previous change of vref
    ,ADC2_CHS,CURRENT_READINGS,CURRENT_DISCARD
    ,ADC3_CHS,CURRENT_READINGS,CURRENT_DISCARD
    ,ADC4_CHS,CURRENT_READINGS,CURRENT_DISCARD
#ifdef ADC5_CHS
    ,ADC5_CHS,CURRENT_READINGS,CURRENT_DISCARD
#endif
#ifdef ADC6_CHS
    ,ADC6_CHS,CURRENT_READINGS,CURRENT_DISCARD
#endif
#ifdef ADC7_CHS
    ,ADC7_CHS,CURRENT_READINGS,CURRENT_DISCARD
#endif
#ifdef ADC8_CHS
    ,ADC8_CHS,CURRENT_READINGS,CURRENT_DISCARD
#endif
#ifdef ADC9_CHS
    ,ADC9_CHS,VOLTAGE_READINGS,VOLTAGE_DISCARD
#endif
};
#define NUMBER_ADC_CHANNELS (sizeof(g_ChannelInfo)/sizeof(ADCINFO))

typedef struct tagADCRESULT
{
    int m_nReading;
    BYTE m_bValid;
}ADCRESULT;

#define NUMBER_ADC_RESULTS (NUMBER_ADC_CHANNELS)

#define TRUE 1
#define FALSE 0

ADCRESULT g_ADCResults[NUMBER_ADC_RESULTS] =
{
     0, FALSE
    ,0, FALSE
    ,0, FALSE
    ,0, FALSE
#ifdef ADC5_CHS
    ,0, FALSE
#endif
#ifdef ADC6_CHS
    ,0, FALSE
#endif
#ifdef ADC7_CHS
    ,0, FALSE
#endif
#ifdef ADC8_CHS
    ,0, FALSE
#endif
#ifdef ADC9_CHS
    ,0, FALSE
#endif
};
#ifdef PROCESS_BATTERY
BYTE ServiceBattery()
{
    ADCRESULT* pR = g_ADCResults + VOLTAGE_CHANNEL;
    // we assume a board only has one battery connected
    // which is the one whose voltage is measured.
    if ( g_bManualCharging == 0)
    {
        if ( g_bCharging )
        {
            if ( g_nFITInterval == 0 )
            {
                // not a valid interval
                g_nFITInterval = 2101;
            }
            if ( g_nFITInterval >= FREE_CHARGING_INTERVAL )
            {
                long TimeSinceStartCharging = (GetTickCount() - g_TimeStartCharging) * TICKS_PER_SECOND;
                if ( g_nFITInterval >= STOP_CHARGING_INTERVAL )
                {
                    if ( pR->m_bValid && pR->m_nReading > KEEP_CHARGING_VOLTAGE )
                    {
                        StopCharging(SWITCH_REASON_STOP_CHARGING_INTERVAL);
                        return 3;
                    }
                }
                else if ( pR->m_bValid && pR->m_nReading > STOP_CHARGING_UNLESS_FREE_VOLTAGE && TimeSinceStartCharging > MIN_CHARGING_PERIOD )
                {
                    StopCharging(SWITCH_REASON_STOP_CHARGING_UNLESS_FREE);
                    return 2;
                }
            }
        }
        else
        {
            if ( pR->m_bValid && pR->m_nReading < MUST_CHARGE_VOLTAGE )
            {
                StartCharging(SWITCH_REASON_MUST_CHARGE_VOLTAGE);
                return 2;
            }
            else if ( g_nFITInterval < FREE_CHARGING_INTERVAL )
            {
                StartCharging(SWITCH_REASON_FREE_CHARGING_INTERVAL);
                return 1;
            }
            else if ( g_nFITInterval < START_CHARGING_INTERVAL && pR->m_bValid && pR->m_nReading <= START_CHARGING_VOLTAGE )
            {
                StartCharging(SWITCH_REASON_START_CHARGING);
                return 3;
            }
        }
    }
    return 0;
}
#endif
//BYTE g_nAdcResultToSend = 0;

void SetupADC(void)
{
//  SetAllADCPins(1);
//    ADCON2bits.ADCS = 4;    // Conversion clock is FOSC/4
    ADCON2bits.ADCS = 2;   // conversion clock is FOSC/32. This gives TAD of about 2 microseconds at 16MHz
    //ADCON2bits.ADCS = 5;     // conversion clock is FOSC/16. This gives TAD of about 1 microseconds at 16Mhz
    ADCON2bits.ACQT = 4;    
    // Acquisition time needs to be sufficient - one source suggests 2 * minimum 
    // minimum being 7.45 microseconds (e.g. 14.9 microseconds)
    // ACQT=4 gives an acquisition time of 8 TAD = 16 microseconds.
    ADCON1 = 0;
    ADCON2bits.ADFM = 1; // right justified result
    ADCON0bits.ADON = 1; // turn on AD converter
    // set up fixed voltage reference for
    // 2V to measure battery voltage
    // 1V to measure current sense resistors
    // Fixed voltage reference is being used need to wait for it to stabilise
    VREFCON0bits.FVRS = 0x1;
    VREFCON0bits.FVREN = 1;
    while(!VREFCON0bits.FVRST);
}
void StartADC()
{
    //int count;
    ADCON1bits.PVCFG = 2;   // use internal voltage reference
    if ( g_CurrentADC == 0 )
    {
        VREFCON0bits.FVRS = 0x1;
// is the following needed doco says that FVRST test is needed after 
// setting FVREN rather than FVRS.
        VREFCON0bits.FVREN = 1;
        while(!VREFCON0bits.FVRST);
    }
    if ( g_CurrentADC >= VOLTAGE_CHANNEL )     // being used for voltage measurement
    {
        VREFCON0bits.FVRS = 0x2;
// is the following needed doco says that FVRST test is needed after 
// setting FVREN rather than FVRS.
        VREFCON0bits.FVREN = 1;
        while(!VREFCON0bits.FVRST);
    }
    ADCON1bits.TRIGSEL = 0; // ignore the CTMU trigger
    PIR1bits.ADIF = 0;
    CTMUCONHbits.CTMUEN = 0;
    ADCON0bits.CHS = g_ChannelInfo[g_CurrentADC].m_Channel;
	ADCON0bits.GO =    1;       //Set Go bit to begin A/D conversion
}
unsigned char HasADCResult(void)
{
    return PIR1bits.ADIF;
}
int GetADCResult(void)
{
    return ((int)ADRESH << 8) | (int)ADRESL;
}
BYTE g_HaveADC = 0xFF;
BYTE g_nADCCount = 0;
long g_nTotal = 0;
BOOL CheckADCRange(BYTE nResult, int nReading)
{
    if ( nResult == VOLTAGE_CHANNEL )
    {
        if ( nReading < SILLY_VOLTAGE_MIN || nReading > SILLY_VOLTAGE_MAX )
        {
            return FALSE;
        }
    }
    return TRUE;
}
void SendReadings();
void ServiceADC(void)
{
    if ( g_HaveADC )
    {
       StartADC();
       g_HaveADC = 0;
    }
    else if ( PIR1bits.ADIF )
    {
        ADCRESULT* pADCResult = g_ADCResults + g_CurrentADC;
        ADCINFO* pADCInfo = g_ChannelInfo + g_CurrentADC;
        //
        // Consider the following remaining comment:
        // See if we have an ADC result yet but only get these or start ADC
        // at the start of a duty cycle
        //
        int nADCResult = GetADCResult();
        g_nADCCount++;
        if ( g_nADCCount > pADCInfo->m_Discard )
        {
            g_nTotal += nADCResult;
        }
        if ( g_nADCCount >= pADCInfo->m_Count )
        {
            nADCResult = g_nTotal / (g_nADCCount - pADCInfo->m_Discard);
            g_nTotal = 0;
            g_nADCCount = 0; 
            BYTE nLastChannel = g_CurrentADC;
            if ( nLastChannel < LED_COUNT )
            {
                // All ADC are currents for lights
                // measured across a 0.1R current sense resistor
                // reference voltage is 1.024V represented by 1024
                // I = V/R
                // so current in milliamps is nADCResult * 10
                // This means that 1 becomes 10ma.
                // and maximum current measurable is 10.24 amps.
                // To get a more accurate result in the low range amplification would be required.
                nADCResult *= 10;
            }
            else
            {
                // Voltage is measured across a potential divider.
                // Total resistance across voltage being measured = 1.1M
                // Tap point for ADC 0.1M
                // Reference voltage is 
                nADCResult *= 22;
            }
            // check result is in range
            if ( CheckADCRange(nLastChannel, nADCResult ) )
            {
                pADCResult->m_nReading = nADCResult;
                pADCResult->m_bValid = TRUE;
#ifdef PROCESS_BATTERY
                if ( nLastChannel == VOLTAGE_CHANNEL )
                {
                    ServiceBattery();
                }
#endif
            }
            g_CurrentADC++;
            if ( g_CurrentADC >= NUMBER_ADC_CHANNELS )
            {
                g_CurrentADC = 0;
            }
        }
        g_HaveADC = 1;
    }  
}
#endif
#if !defined(AMPLIFIER) && !defined(ANALOG_AUDIO)
void SetLEDValue(BYTE nLed, BYTE bOn)
{
    switch(nLed)
    {
    case 0:
        LED1_LAT = bOn;
        break;
    case 1:
        LED2_LAT = bOn;
        break;
    case 2:
        LED3_LAT = bOn;
        break;
    case 3:
        LED4_LAT = bOn;
        break;
#if LED_COUNT > 4
    case 4:
        LED5_LAT = bOn;
        break;
    case 5:
        LED6_LAT = bOn;
        break;
    case 6:
        LED7_LAT = bOn;
        break;
    case 7:
        LED8_LAT = bOn;
        break;
#endif
    }
}
// set up for PWM using timer 2
void StartPWM(void)
{
    // DIsplay test pattern
    LED1_TRIS = 0;
    LED2_TRIS = 0;
    LED3_TRIS = 0;
    LED4_TRIS = 0;
    LED1_LAT = 0;
    LED2_LAT = 0;
    LED3_LAT = 0;
    LED4_LAT = 0;
#if LED_COUNT > 4
    LED5_TRIS = 0;
    LED6_TRIS = 0;
    LED7_TRIS = 0;
    LED8_TRIS = 0;
    LED5_LAT = 0;
    LED6_LAT = 0;
    LED7_LAT = 0;
    LED8_LAT = 0;
#endif
    int n = 0;
    int nLED;
#ifndef LIGHT_CONTROLLER
    for( nLED = 0; nLED < LED_COUNT; nLED++)
    {
        BYTE bOn = 0;
        for( n = 0; n < 8; n++ )
        {
            bOn = !bOn;
            SetLEDValue(nLED, bOn);
            int nC = 5000;
            int nLTC = GetTickCount();
            //while(1);
            while( nC > 0  )
            {
                if ( GetTickCount() != nLTC )
                {
                    nC--;
                    nLTC = GetTickCount();
                }
            }
        }
    }
#endif
//  Disable CCPx pin driver
    LED1_PWM_TRIS = 1;
    LED2_PWM_TRIS = 1;
    LED3_PWM_TRIS = 1;
    LED4_PWM_TRIS = 1;
#if LED_COUNT >= 8
    LED5_PWM_TRIS = 1;
    LED6_PWM_TRIS = 1;
    LED7_PWM_TRIS = 1;
    LED8_PWM_TRIS = 1;
#endif
// Set up timer 2
    T2CONbits.TMR2ON = 0; // disable timer
    T2CONbits.T2CKPS = 0; // clock prescaler 1
    T2CONbits.T2OUTPS = 0;  // use 1 times prescaler
    TMR2 = 0;
// Select timer 2 for each LED
    LED1_TSEL = 0;
    LED2_TSEL = 0;
    LED3_TSEL = 0;
    LED4_TSEL = 0;
#if LED_COUNT > 4
    LED5_TSEL = 0;
    LED6_TSEL = 0;
    LED7_TSEL = 0;
    LED8_TSEL = 0;
#endif
// Set period for PWM
    PR2 = 0xFF;

// Set each PWM mode
// Configure CCP module for each LED
// Load duty cycle
// LED1_DCB
#if LED1_USE_PWM == 1
    LED1_PWMMODE = 0;
    LED1_CCPM = 0xC;
    LED1_CPPRL = 0xFF;
    LED1_DCB = 0x3;
#endif
#if LED2_USE_PWM == 1
    LED2_PWMMODE = 0;
    LED2_CCPM = 0xC;
    LED2_CPPRL = 0xFF;
    LED2_DCB = 0x3;
#endif
#if LED3_USE_PWM == 1
    LED3_PWMMODE = 0;
    LED3_CCPM = 0xC;
    LED3_CPPRL = 0xFF;
    LED3_DCB = 0x3;
#endif
#if LED4_USE_PWM == 1
    LED4_PWMMODE = 0;
    LED4_CCPM = 0xC;
    LED4_CPPRL = 0xFF;
    LED4_DCB = 0x3;
#endif
#if LED_COUNT > 4
#if LED5_USE_PWM == 1
    LED5_PWMMODE = 0;
    LED5_CCPM = 0xC;
    LED5_CPPRL = 0xFF;
    LED5_DCB = 0x3;
#endif
#if LED6_USE_PWM == 1
    LED6_PWMMODE = 0;
    LED6_CCPM = 0xC;
    LED6_CPPRL = 0xFF;
    LED6_DCB = 0x3;
#endif
#if LED7_USE_PWM == 1
    LED7_PWMMODE = 0;
    LED7_CCPM = 0xC;
    LED7_CPPRL = 0xFF;
    LED7_DCB = 0x3;
#endif
#if LED8_USE_PWM == 1
    LED8_PWMMODE = 0;
    LED8_CCPM = 0xC;
    LED8_CPPRL = 0xFF;
    LED8_DCB = 0x3;
#endif
#endif
// Start timer
    T2CONbits.TMR2ON = 1; // turn timer on.
    PIE1bits.TMR2IE = 1;
    LED1_PWM_TRIS = 0;
    LED2_PWM_TRIS = 0;
    LED3_PWM_TRIS = 0;
    LED4_PWM_TRIS = 0;
#if LED_COUNT >= 8
    LED5_PWM_TRIS = 0;
    LED6_PWM_TRIS = 0;
    LED7_PWM_TRIS = 0;
    LED8_PWM_TRIS = 0;
#endif
}
#ifndef MANUAL_PWM_USE_TIMER
void SetLEDManual(BYTE nLed, BYTE bOn)
{
    switch(nLed)
    {
    case 0:
#if (LED1_USE_PWM == 0 )
        LED1_LAT = bOn;
#endif
        break;
    case 1:
#if (LED2_USE_PWM == 0 )
        LED2_LAT = bOn;
#endif
        break;
    case 2:
#if (LED3_USE_PWM == 0 )
        LED3_LAT = bOn;
#endif
        break;
    case 3:
#if (LED4_USE_PWM == 0 )
        LED4_LAT = bOn;
#endif
        break;
#if LED_COUNT > 4
        case 4:
#if (LED5_USE_PWM == 0)
        LED5_LAT = bOn;
#endif
        break;
#endif
#if LED_COUNT > 5
        case 5:
#if (LED6_USE_PWM == 0)
        LED6_LAT = bOn;
#endif
        break;
#endif
#if LED_COUNT > 6
        case 6:
#if (LED6_USE_PWM == 0)
        LED6_LAT = bOn;
#endif
        break;
#endif
#if LED_COUNT > 7
        case 7:
#if (LED7_USE_PWM == 0)
        LED7_LAT = bOn;
#endif
        break;
#endif
    }
}
void DoManualPWM(void)
{
    BYTE n;
#ifndef MANUAL_PWM_USE_TIMER
    g_dutyCycle++;
    if ( g_dutyCycle == 0)
    {
        g_resetDutyCycle = 1;
    }
#endif
    for( n = 0; n < LED_COUNT; n++ )
    {
        if ( g_resetDutyCycle && LEDLevels[n] > 0 )
        {
            SetLEDManual(n, 1);
        }
        if (g_LEDLevels[n] <= g_dutyCycle )
        {
            SetLEDManual(n, 0);
        }
    }
    resetDutyCycle = 0;
}
#endif
#endif
#if ANALOG_AUDIO
BYTE g_OldLevel[4];
void I2CMasterWait()
{
    while( (SSP2STAT & 0x4 ) || (SSP2CON2 & 0x1F) );
    PIR3bits.SSP2IF = 0;
}
void I2CStart()
{
    I2CMasterWait();
    SSP2CON2bits.SEN = 1;
}
void I2CStop()
{
    I2CMasterWait();
    SSP2CON2bits.PEN = 1;
}
void I2CSend(BYTE by)
{
    I2CMasterWait();
    SSP2BUF = by;
}
void SendI2C(BYTE* byData, BYTE nLen)
{
//    I2CSend(0);    // Send address
    SSPSTATbits.D_NOT_A = 0; // indicates address ?
    //  7 bit address is 0x54 + aa + rw
    //  aa is 0 to 3 depending on device A0, A1 
    //  rw is read/write bit 0 = write. 1 = read
    // keep trying address until there is a response
    BYTE n;
    for(n = 0; n < 100; n++)
    {
        I2CStart();
        I2CSend(0x54);
        while(SSPSTATbits.BF);
        if ( SSP2CON2bits.ACKSTAT)
        {
            SSP2CON2bits.PEN = 1;            
            SendBytes("NAKed",5);
        }
        else
        {
            break;
        }
    }
    SSPSTATbits.D_NOT_A = 1; // indicates data ?
    while( nLen > 0)
    {
        I2CSend(*byData);
        byData++;
        nLen--;
    }
    I2CStop();
}
void SetChannel(BYTE byPort, BYTE byChannel)
{
    byChannel--;
    switch(byPort)
    {
    case 0:
        CHANNEL_0_A_LAT = byChannel & 1;
        CHANNEL_0_B_LAT = (byChannel >> 1) & 1;
        break;
    case 1:
        CHANNEL_1_A_LAT = byChannel & 1;
        CHANNEL_1_B_LAT = (byChannel >> 1) & 1;
        break;
    }
}
#if USE_GENERAL_COMMAND
void SetVolume(BYTE byPort, BYTE byVolume)
{
    // Volume control uses pins 22,23 
    // 22 SCL2 connects to pot 5 SCL should have pull up but does not. will weak pull up on port B work?
    // 23 SDA2 connects to pot 6 SDA has pull up.
    if ( byVolume == 0 )
    {
        if ( byPort == 0 )
        {
            CHANNEL_0_SW_LAT = 0;
        }
        else
        {
            CHANNEL_1_SW_LAT = 0;
        }
    }
    BYTE byCommand[2];

// If using general command format is different
// See MCP444x / 446x datasheet section 6.2.7
//  Byte 0 address (all 0)
//  Byte 1 
//    7:4 Command
//        1000 write next byte to volatile wiper 0
//        1001 write next byte to volatile wiper 1
//        1100 write next byte to TCON register
//    3:1
//        00d  write to wiper (only if 7:4 is 1000 or 1001        
//             d is top bit of 9 bit data value
//        01x  increment wiper
//        10x  decrement wiper
//    0   Always 0
//  Byte 2 volume (or other data)
    byCommand[0] = 0x80 & (byPort == 0 ? 0 : 0x10 );
    byCommand[1] = byVolume;
    SendI2C(byCommand, 2);
    byCommand[0] = 0x80 & (byPort == 0 ? 0 : 0x10 );
// problem we want to set pairs of wipers either 0,2 or 1,3
    SendI2C(byCommand, 2);
    if ( byVolume != 0 )
    {
        if ( byPort == 0 )
        {
            CHANNEL_0_SW_LAT = 1;
        }
        else
        {
            CHANNEL_1_SW_LAT = 1;
        }
    }
}
#else
void SetVolume(BYTE byPort, BYTE byVolume)
{
    // Volume control uses pins 22,23 
    // 22 SCL2 connects to pot 5 SCL should have pull up but does not. will weak pull up on port B work?
    // 23 SDA2 connects to pot 6 SDA has pull up.
    if ( byVolume == 0 )
    {
        if ( byPort == 0 )
        {
            CHANNEL_0_SW_LAT = 0;
        }
        else
        {
            CHANNEL_1_SW_LAT = 0;
        }
    }
    // Command byte
    // 7:4 Data address
    // 0 Volatile wiper 0
    // 1 Volatile wiper 1
    // 2 NV Wiper 0
    // 3 NV Wiper 1
    // 4 TCON0
    // 5 Status
    // 6 Volatile wiper 2
    // 7 Volatile wiper 3
    // 8 NV Wiper 2
    // 9 NV Wiper 3
    // 10 TCON1
    // 11 - 15 EEPROM
    // 3:2 0 for write 3 for read
    // 1:0 1 if byVolume is 255 to send full signal through otherwise 0 - don't bother with this!

    // Data byte
    // If byVolume = 255 then 0 else byVolume
    // Rest of data
    BYTE byCommand[2];
    byCommand[0] = (byPort == 0 ? 1 : 0) << 4;
    byCommand[1] = byVolume;
    SendI2C(byCommand, 2);
    byCommand[0] = (byPort == 0 ? 7 : 6) << 4;
    SendI2C(byCommand, 2);
    if ( byVolume != 0 )
    {
        if ( byPort == 0 )
        {
            CHANNEL_0_SW_LAT = 1;
        }
        else
        {
            CHANNEL_1_SW_LAT = 1;
        }
    }
}
#endif
void SetLED(BYTE byLED, BYTE byLevel)
{
    if ( byLevel != g_OldLevel[byLED - 1] )
    {
        // TODO set volume and channel 
        switch(byLED)
        {
        case 1:
            // 1 Amp1 volume , also 0 = 0ff anything else = on
            SetVolume(0, byLevel);
            break;
        case 2:
            // 2 Amp1 channel
            SetChannel(0, byLevel);
            break;
        case 3:
            // 3 Amp2 volume
            SetVolume(1, byLevel);
            break;
        case 4:
            // 4 Amp2 channel
            SetChannel(1, byLevel);
            break;
        }
        g_OldLevel[byLED-1] = byLevel;
    }
}
void InitAnalogAudio()
{
    g_OldLevel[0] = 0;
    g_OldLevel[1] = 0;
    g_OldLevel[2] = 0;
    g_OldLevel[3] = 0;
    CHANNEL_0_A_ANSEL = 0;
    CHANNEL_0_B_ANSEL = 0;
    CHANNEL_1_A_ANSEL = 0;
    CHANNEL_1_B_ANSEL = 0;
    CHANNEL_0_SW_ANSEL = 0;
    CHANNEL_1_SW_ANSEL = 0;
    VOLUME_ADR_0_ANSEL = 0;
    VOLUME_ADR_1_ANSEL = 0;
    VOLUME_RESET_ANSEL = 0;
    VOLUME_SCL_ANSEL = 0;
    VOLUME_SDA_ANSEL = 0;

    CHANNEL_0_A_TRIS = 0;
    CHANNEL_0_B_TRIS = 0;
    CHANNEL_1_A_TRIS = 0;
    CHANNEL_1_B_TRIS = 0;
    CHANNEL_0_SW_TRIS = 0;
    CHANNEL_1_SW_TRIS = 0;
    VOLUME_ADR_0_TRIS = 0;
    VOLUME_ADR_0_LAT = 0;
    VOLUME_ADR_1_TRIS = 0;
    VOLUME_ADR_1_LAT = 0;
    VOLUME_RESET_TRIS = 0;
    VOLUME_RESET_LAT = 0;
    VOLUME_ADR_0_LAT = 0;
    VOLUME_ADR_1_LAT = 0;
    VOLUME_RESET_LAT = 0;
    VOLUME_SCL_TRIS = 1;
    VOLUME_SDA_TRIS = 1;

    VOLUME_RESET_LAT = 1;
    
    // Set up I2C
    SSP2STAT = 0;
    SSP2STATbits.SMP = 1;          // slew control disable
    SSP2STATbits.CKE = 1;          // Enable input logic so that thresholds are commpliant with SMbus spec.
    SSP2CON1 = 0;
    SSP2CON1bits.SSPEN = 1;       // Enable device
    SSP2CON1bits.SSPM = 0b1000;  // I2C master mode clock is FOsc / 4
    SSP2CON2 = 0;
    SSP2ADD = 0x27;                // 100khz baud rate for 16mhz clock
    SetVolume(0,0);
    SetVolume(1,0);
    SetChannel(0,1);
    SetChannel(1,1);
}
#elif 0
typedef enum
{
    AMP_OFF = 0,
    AMP_WAIT_UNMUTE = 1,
    AMP_ON = 2,
    AMP_WAIT_PWR_OFF = 3
}AMPSTATE;
typedef struct tagAMPSTATUS
{
    AMPSTATE m_nState;
    unsigned long m_tWait;
}AMPSTATUS;
AMPSTATUS g_AmpStatus[LED_COUNT];
void SetMute(unsigned char byAmp, unsigned char byLevel)
{
    switch(byAmp)
    {
    case 1:
        AMP1_MUTE_LAT = byLevel;
        break;
    case 2:
        AMP2_MUTE_LAT = byLevel;
        break;
    case 3:
        AMP3_MUTE_LAT = byLevel;
        break;
    case 4:
        AMP4_MUTE_LAT = byLevel;
        break;
    case 5:
        AMP5_MUTE_LAT = byLevel;
        break;
    case 6:
        AMP6_MUTE_LAT = byLevel;
        break;
    case 7:
        AMP7_MUTE_LAT = byLevel;
        break;
    case 8:
        AMP8_MUTE_LAT = byLevel;
        break;
    }
}
void SetStby(unsigned char byAmp, unsigned char byLevel)
{
    switch(byAmp)
    {
    case 1:
        AMP1_STBY_LAT = byLevel;
        break;
    case 2:
        AMP2_STBY_LAT = byLevel;
        break;
    case 3:
        AMP3_STBY_LAT = byLevel;
        break;
    case 4:
        AMP4_STBY_LAT = byLevel;
        break;
    case 5:
        AMP5_STBY_LAT = byLevel;
        break;
    case 6:
        AMP6_STBY_LAT = byLevel;
        break;
    case 7:
        AMP7_STBY_LAT = byLevel;
        break;
    case 8:
        AMP8_STBY_LAT = byLevel;
        break;
    }
}
void SetAmplifier(BYTE byAmp, BYTE byLevel)
{
    // byLevel is 0 for off otherwise on
    // this means that pin values are inverted as 1 is mute and standby is 1 , on is 0
    if ( byLevel == 0 )
    {
        g_AmpStatus[byAmp-1].m_nState = AMP_WAIT_PWR_OFF;
        g_AmpStatus[byAmp-1].m_tWait = GetTickCount() + AMP_DELAY;
        SetMute(byAmp, 1);
    }
    else
    {
        g_AmpStatus[byAmp-1].m_nState = AMP_WAIT_UNMUTE;
        g_AmpStatus[byAmp-1].m_tWait = GetTickCount() + AMP_DELAY;
        SetStby(byAmp, 0);
    }
}
void ServiceAmplifier()
{
    unsigned char n;
    for(n = 0; n < LED_COUNT; n++ )
    {
        AMPSTATUS* pStatus = g_AmpStatus + n;
        if ( pStatus->m_nState == AMP_WAIT_PWR_OFF || pStatus->m_nState == AMP_WAIT_UNMUTE )
        {
            // See if time has elapsed
            unsigned long t = GetTickCount();
            if ( t > pStatus->m_tWait)
            {
                // Deal with wrap around case.
                if ( !(pStatus->m_tWait < AMP_DELAY && t + AMP_DELAY < t) )
                {
                    if ( pStatus->m_nState == AMP_WAIT_PWR_OFF )
                    {
                        SetStby(n+1, 1);
                        pStatus->m_nState = AMP_OFF;
                    }
                    else if ( pStatus->m_nState == AMP_WAIT_UNMUTE )
                    {
                        SetMute(n+1, 0);
                        pStatus->m_nState = AMP_ON;
                    }
                }
            } 
        }
    }
}
#else
void SetLED(BYTE byLED, BYTE byLevel)
{
// now change this to use PWM
// Each LED needs CCPRxL and DCxB<1:0> loading with PWM duty cycle

    BYTE bOn = byLevel == 0 ? 0 : 1;
    g_LEDLevels[byLED-1] = byLevel;
    switch(byLED)
    {
    case 1:
#if ( LED1_USE_PWM == 1 )
        LED1_TRIS = 1;
        LED1_CPPRL = byLevel;
        LED1_DCB = byLevel == 0 ? 0 :3;
        LED1_TRIS = 0;
#elif ( LED1_USE_PWM == 2 )
        LED1_LAT = 0;
#else
        LED1_LAT = bOn;
#endif
        break;
    case 2:
#if ( LED2_USE_PWM == 1)
        LED2_TRIS = 1;
//        LED2_CCPM = 0;
        LED2_CPPRL = byLevel;
        LED2_DCB = byLevel == 0 ? 0 :3;
//        LED2_CCPM = 0xF;
        LED2_TRIS = 0;
#elif ( LED2_USE_PWM == 2 )
        LED2_LAT = 0;
#else
        LED2_LAT = bOn;
#endif
        break;
    case 3:
#if ( LED3_USE_PWM == 1)
        LED3_TRIS = 1;
        LED3_CPPRL = byLevel;
        LED3_DCB = byLevel == 0 ? 0 :3;
        LED3_TRIS = 0;
#elif ( LED3_USE_PWM == 2 )
        LED3_LAT = 0;
#else
        LED3_LAT = bOn;
#endif
        break;
    case 4:
#if ( LED4_USE_PWM == 1)
        LED4_TRIS = 1;
        LED4_CPPRL = byLevel;
        LED4_DCB = byLevel == 0 ? 0 :3;
        LED4_TRIS = 0;
#elif ( LED4_USE_PWM == 2 )
        LED4_LAT = 0;
#else
        LED4_LAT = bOn;
#endif
        break;
#if LED_COUNT > 4
    case 5:
#if ( LED5_USE_PWM == 1)
        LED5_TRIS = 1;
        LED5_CPPRL = byLevel;
        LED5_DCB = byLevel == 0 ? 0 :3;
        LED5_TRIS = 0;
#elif ( LED5_USE_PWM == 2 )
        LED5_LAT = 0;
#else
        LED5_LAT = bOn;
#endif
        break;
    case 6:
#if ( LED6_USE_PWM == 1)
        LED6_TRIS = 1;
        LED6_CPPRL = byLevel;
        LED6_DCB = byLevel == 0 ? 0 :3;
        LED6_TRIS = 0;
#elif ( LED6_USE_PWM == 2 )
        LED6_LAT = 0;
#else
        LED6_LAT = bOn;
#endif
        break;
    case 7:
#if ( LED7_USE_PWM == 1)
        LED7_TRIS = 1;
        LED7_CPPRL = byLevel;
        LED7_DCB = byLevel == 0 ? 0 :3;
        LED7_TRIS = 0;
#elif ( LED7_USE_PWM == 2 )
        LED7_LAT = 0;
#else
        LED7_LAT = bOn;
#endif
        break;
    case 8:
#if ( LED8_USE_PWM == 1)
        LED8_TRIS = 1;
        LED8_CPPRL = byLevel;
        LED8_DCB = byLevel == 0 ? 0 :3;
        LED8_TRIS = 0;
#elif ( LED8_USE_PWM == 2 )
        LED8_LAT = 0;
#else
        LED8_LAT = bOn;
#endif
        break;
#endif
#if LED_COUNT > 8
    case 9:
        LED9_LAT = bOn;
        break;
    case 10:
        LED10_LAT = bOn;
        break;
    case 11:
        LED11_LAT = bOn;
        break;
    case 12:
        LED12_LAT = bOn;
        break;
    case 13:
        LED13_LAT = bOn;
        break;
    case 14:
        LED14_LAT = bOn;
        break;
    case 15:
        LED15_LAT = bOn;
        break;
    case 16:
        LED16_LAT = bOn;
        break;
#endif
    }
}
#endif
BYTE HexChar(BYTE by)
{
    if ( by >= 0 && by < 10 )
    {
        return by + '0';
    }
    else if ( by >= 10 && by < 16 )
    {
        return by - 10 + 'A';
    }
    return '?';
}
void SendReady(void)
{
    char sz[8];
    sz[0] = '\r';
    sz[1] = '\n';
    sz[2] = '$';
    sz[3] = '0';
    sz[4] = 'x';
    sz[5] = HexChar(SWITCH_ID >> 4);
    sz[6] = HexChar(SWITCH_ID & 0xF);
    sz[7] = '>';
    SendBytes(sz,8);
#ifdef ECHO_UART1
    SendBytes1(sz,8);
#endif
}
char szTransferCMD[10];
BYTE nTransferCMD = 0;
#ifdef FORWARD_UART2
BYTE nTX1State = 0;
BYTE byCmd = 0;
void TransferRC2toTX1(void)
{
    TXSTA1bits.TXEN=1;
    while( TestBuffer( rcHead2, rcTail2 )) 
    {
        BYTE ch = PopBuffer( szRCBUF2, &rcHead2, &rcTail2 );
        // We only send commands through!
        if ( ch == '=')
        {
            nTX1State = 1;
            szTransferCMD[0] = ch;
            nTransferCMD = 1;
        }
        // TODO also need to send through status!
        else if ( nTX1State > 0)
        {
            // Ensure commands are always sent whole to avoid not interleaved between boards.
            if ( nTransferCMD < sizeof(szTransferCMD) - 1)
            {
                BYTE nPin = 0;
                switch(nTX1State)
                {
                case 1:
                    // For status reports toggle when voltage has been sent.
                    // only send alternate status reports through.
                    if ( ch == 'V' || ch == 'I' || ch == 'F' || ch == 'T' || ch == 'L' || ch == 'Z')
                    {
                        byCmd = ch;
                        nTX1State = 2;
                    }
                    break;
                case 2:
                    // Pin must be > LED_COUNT
                    if ( ch >= '0' && ch <= '9')
                    {
                        nPin = ch - '0';
                    }
                    else if ( ch >= 'A' && ch <= 'Z')
                    {
                        nPin = ch - 'A' + 10;
                    }
                    if ( nPin <= LED_COUNT )
                    {
                        nTX1State = 0;
                    }
                    else
                    {
                        nTX1State = 3;
                    }
                    break;
                case 3:
                    break;
                }
                if ( nTX1State > 0)
                {
                    szTransferCMD[nTransferCMD] = ch;
                    nTransferCMD++;
                }
            }
            if ( ch == '\r' )
            {
                if ( nTX1State > 0)
                {
                    SendBytes(szTransferCMD, nTransferCMD);
                }
                nTransferCMD = 0;
                nTX1State = 0;
            }
        }
    }
}
#endif
#ifdef FORWARD_UART1
BYTE nTX2State = 0;
void TransferRC1toTX2(void)
{
    TXSTA2bits.TXEN=1;
    while( TestBuffer( rcHead1, rcTail1 )) 
    {
        BYTE ch = PopBuffer( szRCBUF1, &rcHead1, &rcTail1 );
        // We only send commands through!
        if ( ch == '=')
        {
            nTX2State = 1;
            nTransferCMD = 0;
        }
        if ( nTX2State == 1)
        {
            // Ensure commsnds are always sent whole
            // to avoid not interleaved between boards.
            if ( nTransferCMD < sizeof(szTransferCMD) - 1)
            {
                if ( ch >= '0' && ch <= '9')
                {
                    if ( (ch - '0') <= LED_COUNT )
                    {
                        nTX2State = 0;
                    }
                }
                szTransferCMD[nTransferCMD] = ch;
                nTransferCMD++;
            }
            if ( ch == '\r')
            {
                SendBytes(szTransferCMD, nTransferCMD);
                nTransferCMD = 0;
                nTX2State = 0;
            }
        }
    }
}
#endif
void SendLevel2(BYTE curLED, BYTE curLevel)
{
    char sz[8];
    sz[0] = 'L';
    sz[1] = '0' + curLED;
    sz[2] = HexChar(curLevel >> 4);
    sz[3] = HexChar(curLevel & 0xF);
    sz[4] = '\r';
    sz[5] = '\n';
    sz[6] = 0;
    SendBytes2(sz,6);
}
void SendHex1(char ch)
{
    char sz[5];
    sz[0] = '{';
    sz[1] = HexChar(ch >> 4);
    sz[2] = HexChar(ch & 0xF);
    sz[3] = '}';
    sz[4] = 0;
    SendBytes1(sz,4);
}
#ifdef LIGHT_CONTROLLER
BYTE EncodePin(BYTE nPin)
{
    if ( nPin < 10 )
    {
        return '0' + nPin;
    }
    else if ( nPin < 36 )
    {
        return 'A' + nPin - 10;
    }
    return '?';
}
#if !defined(AMPLIFIER)
void SendADCReading(BYTE bySwitch, BYTE byCommand, ADCRESULT* pResult)
{
    char szCMD[10];
    szCMD[0] = '=';
    szCMD[1] = byCommand;
    szCMD[2] = EncodePin(bySwitch);
    if ( pResult->m_bValid )
    {
        int usReading = pResult->m_nReading;
        szCMD[3] = HexChar(usReading >> 12);
        szCMD[4] = HexChar((usReading >> 8) & 0xF);
        szCMD[5] = HexChar((usReading >> 4) & 0xF);
        szCMD[6] = HexChar( usReading & 0xF);
    }
    else
    {
        szCMD[3] = '#';
        szCMD[4] = '#';
        szCMD[5] = '#';
        szCMD[6] = '#';
    }
    szCMD[7] = '\r';
    szCMD[8] = '\n';
    szCMD[9] = 0;
    SendBytes(szCMD,9);
}
void SendVoltageReading()
{
    SendReady();
    SendADCReading(LED_OFFSET + 0, 'V', g_ADCResults + VOLTAGE_CHANNEL);
    SendReady();
}
void SendCurrentReading(BYTE bySwitch, ADCRESULT* pResult)
{
    SendReady();
    SendADCReading(bySwitch + LED_OFFSET, 'I', pResult);
    SendReady();
}
#ifdef PROCESS_BATTERY
void SendCharging()
{
    char szCMD[7];
    szCMD[0] = '=';
    szCMD[1] = g_bCharging ? 'Y' : 'N';
    szCMD[2] = EncodePin(CHARGER_PIN + LED_OFFSET);
    szCMD[3] = HexChar(g_LastReason >> 8);
    szCMD[4] = HexChar(g_LastReason & 0xF);
    szCMD[5] = '\r';
    szCMD[6] = '\n';
    SendBytes(szCMD,7);
}
#endif
void SendReadings()
{
    BYTE n;
    SendReady();
    ADCRESULT version;
    version.m_bValid = 1;
    version.m_nReading = VERSION << 8 | SOFTWARE_VERSION;
    SendADCReading(LED_OFFSET + 1, 'v', &version);
    
    for( n = 0; n < LED_COUNT; n++ )
    {
        SendADCReading(LED_OFFSET + (n + 1), 'I', g_ADCResults + n);
    }
#ifdef PROCESS_BATTERY
    ADCRESULT FIT;
    FIT.m_bValid = 1;
    FIT.m_nReading = g_nFITInterval;
    SendADCReading(LED_OFFSET + 2, 'F', &FIT);
#endif
    SendADCReading(LED_OFFSET + 1, 'V', g_ADCResults + VOLTAGE_CHANNEL);
#ifdef PROCESS_BATTERY
    if ( g_bChargingChange )
    {
        SendCharging();
        g_bChargingChange = 0;
    }
    
#endif
    SendReady();
}
#endif
#endif
unsigned long g_tNextReady = 0;
unsigned long g_NextTransmitToken = 0;
unsigned int g_NextFITInterval = 2100;
#ifdef RS485_DEVICE
BYTE g_TokenState = 0;
void SetToken(BYTE nToken, long tMilliSeconds)
{
    LED2_LAT = nToken == SWITCH_ID ? 1 : 0;
    g_TransmitToken = nToken;
    g_tTransmitTokenValidTick = GetTickCount() + tMilliSeconds * TICKS_PER_SECOND / 1000;
    g_TokenState = 0;
}
BYTE g_NextSlave = 0;
#endif
BYTE g_byBoard = 0;
typedef enum
{
    PARSE_START = 0,
    PARSE_PIN = 1,
    PARSE_LEVEL_1 = 2,
    PARSE_LEVEL_2 = 3,
    PARSE_FIT = 4,
    PARSE_MASK_1 = 5,
    PARSE_MASK_2 = 6,        
    PARSE_BOARD_1 = 7,
    PARSE_BOARD_2 = 8,
    PARSE_BOARD_3 = 9,
    PARSE_COMPLETE = 99
}PARSE_STATE;
void ReceiveService(void)
{
    // once token time has run out transmission right
    // goes back to master.
#ifdef RS485_DEVICE
    if ( GetTickCount() > g_tTransmitTokenValidTick )
    {
        LED4_LAT=0;
        g_TransmitToken = MASTER_DEVICE;
        g_NextTransmitToken = 0;
        g_TokenState = 0;
    }
#endif
    if ( HaveChar() )
    {
        BYTE sz[2];
        BYTE ch = GetChar();
        sz[0] = ch;
        sz[1] = 0;
#ifdef ECHO_UART1
//        SendHex1(ch);
        SendBytes1(sz,1);
#endif
#ifdef ECHO_UART2
        SendBytes2(sz,1);
#endif
#ifdef RS485_DEVICE
        if ( ch == '{' ) 
        {
            LED3_LAT=1;
            g_NextSlave = 0;
            g_bySlaveState = 1;
        }
        if ( g_bySlaveState > 0)
        {
            switch(g_bySlaveState)
            {
            case 1:
                if ( ch >= '0' && ch <= '9')
                {
                    g_NextSlave = (ch - '0') << 4;
                    g_bySlaveState++;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_NextSlave = (ch - 'A' + 10) << 4;
                    g_bySlaveState++;
                }
                break;
            case 2:
                if ( ch >= '0' && ch <= '9')
                {
                    g_NextSlave |= (ch - '0');
                    g_bySlaveState++;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_NextSlave |= (ch - 'A' + 10);
                    g_bySlaveState++;
                }
                break;
            }
            if ( ch == '}' || ch == '\r')
            {
                if ( g_bySlaveState == 3)
                {
                    g_SlaveDevice = g_NextSlave;
                    if ( g_SlaveDevice == SWITCH_ID )
                    {
                        LED4_LAT=1;
                        SetToken(SWITCH_ID, 10);
                        SendReady();
                    }
                }
                LED3_LAT=0;
                g_bySlaveState = 0;
            }
        }
        else if ( g_SlaveDevice == SWITCH_ID )
#endif
        {
            switch( g_byState )
            { 
            case PARSE_START:
                // L set light level
                // I get current
                // V get voltage
                if ( ch == 'L' || ch == 'I')
                {
                    g_byState = PARSE_PIN;
                    g_byCommand = ch;
                }
                else if ( ch == 'E' )
                {
                    g_byCommand = ch;
                    g_byState = PARSE_BOARD_1;
                }
#ifdef LIGHT_CONTROLLER
                else if ( ch == 'S' || ch == 'V' )
                {
                    g_byCommand = ch;
                    g_byState = PARSE_BOARD_1;
                }
#ifdef PROCESS_BATTERY
                else if ( ch == 'F')
                {
                    g_byState = PARSE_FIT;
                    g_byCommand = ch;
                    g_NextFITInterval = 0;
                }
#endif
#endif
                break;
            case PARSE_PIN:
                if ( ch >= '1' && ch <= '9')
                {
                    g_curLED = ch - '0';
                }
                else if ( ch >= 'A' && ch <= 'Z')
                {
                    g_curLED = ch - 'A' + 10;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                if ( g_byState != PARSE_START && 
                        g_curLED > LED_OFFSET && g_curLED <= (LED_OFFSET + LED_COUNT) )
                {
                    g_curLED -= LED_OFFSET;
                    g_curLevel = 0;
                    if ( g_byCommand == 'L' )
                    {
                        g_byState = PARSE_LEVEL_1;
                    }
                    else
                    {
                        g_byState = PARSE_COMPLETE;
                    }
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_LEVEL_1:
                if ( ch >= '0' && ch <= '9' )
                {
                    g_curLevel =  ( ch - '0' ) << 4;
                    g_byState = PARSE_LEVEL_2;
                }
                else if ( ch >= 'A' && ch <= 'F' )
                {
                    g_curLevel = (ch - 'A' + 10) << 4;
                    g_byState = PARSE_LEVEL_2;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_LEVEL_2:
                if ( ch >= '0' && ch <= '9' )
                {
                    g_curLevel |=  (( ch - '0' ) & 0xF);
                    g_byState = PARSE_COMPLETE;
                }
                else if ( ch >= 'A' && ch <= 'F' )
                {
                    g_curLevel |= ((ch - 'A' + 10) & 0xF);
                    g_byState = PARSE_COMPLETE;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_FIT:
                if ( ch >= '0' && ch <= '9')
                {
                    g_NextFITInterval = 10 * g_NextFITInterval + ch - '0';
                }
                break;
            case PARSE_MASK_1:
                if ( ch >= '0' && ch <= '9')
                {
                    g_curLED = (ch - '0') << 4;
                    g_byState = PARSE_MASK_2;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_curLED = (ch - 'A' + 10) << 4;
                    g_byState = PARSE_MASK_2;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_MASK_2:
                if ( ch >= '0' && ch <= '9')
                {
                    g_curLED |= (ch - '0');
                    g_byState = PARSE_LEVEL_1;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_curLED |= (ch - 'A' + 10);
                    g_byState = PARSE_LEVEL_1;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_BOARD_1:
                if ( ch >= '0' && ch <= '9')
                {
                    g_byBoard = (ch - '0') << 4;
                    g_byState = PARSE_BOARD_2;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_byBoard = (ch - 'A' + 10) << 4;
                    g_byState = PARSE_BOARD_2;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                break;
            case PARSE_BOARD_2:
                if ( ch >= '0' && ch <= '9')
                {
                    g_byBoard |= (ch - '0');
                    g_byState = PARSE_BOARD_3;
                }
                else if ( ch >= 'A' && ch <= 'F')
                {
                    g_byBoard |= (ch - 'A' + 10);
                    g_byState = PARSE_BOARD_3;
                }
                else
                {
                    g_byState = PARSE_START;
                }
                if ( g_byState == PARSE_BOARD_3)
                {
                    if ( g_byBoard != SWITCH_ID)
                    {
                        g_byState = PARSE_START;
                    }
                    else
                    {
                        if ( g_byCommand == 'E')
                        {
                            g_byState = PARSE_MASK_1;
                        }
                        else
                        {
                            g_byState = PARSE_COMPLETE;
                        }
                    }
                }
                break;
            }
#ifndef RS485_DEVICE
            // echo characters received
//            SendBytes(sz, 1);
#endif
#if defined(FORWARD_UART2)
            SendBytes2(sz, 1);
#endif
            if ( ch == '\r' )
            {
                // only acknowledge and act is LED is ours.
                int bReady = 0;
                switch(g_byCommand)
                {
                case 'E':
                    if ( g_byState == PARSE_COMPLETE)
                    {
#if !defined(AMPLIFIER) && !defined(ANALOG_AUDIO)
                        BYTE n;
                        for( n = 0; n < LED_COUNT; n++ )
                        {
                            if ( g_curLED & 1)
                            {
                                SetLED(n+1, g_curLevel);    
                            }
                            g_curLED = g_curLED >> 1;
                        }
#endif
                    }
                    break;
                case 'L':
                    if ( g_byState == PARSE_COMPLETE )
                    {
#ifdef PROCESS_BATTERY
                        if ( g_curLED == CHARGER_PIN )
                        {
                            if ( g_curLevel == 0 && g_bManualCharging == 1)
                            {
                                g_bManualCharging = 0;
                                //ServiceBattery();
                            }
                            else if ( g_curLevel != 0 && g_bManualCharging == 0)
                            {
                                g_bManualCharging = 1;
                                StartCharging(SWITCH_REASON_TOGGLE);
                            }
                        }
                        else
#endif
                        {
                            SetLED(g_curLED, g_curLevel);
                        }
                        bReady = 1;
                    }
                    break;
#if defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
                case 'I':
                    if ( g_byState == PARSE_COMPLETE )
                    {
#ifdef RS485_DEVICE
                        SetToken(SWITCH_ID, 15);
#endif
                        SendCurrentReading(g_curLED, g_ADCResults + (g_curLED - 1));
                    }
                    break;
#ifdef PROCESS_BATTERY
                case 'F':
                    if ( g_byState == PARSE_FIT && g_NextFITInterval > 0)
                    {
                        g_nFITInterval = g_NextFITInterval;
                        ServiceBattery();
                    }
                    break;    
#endif
                case 'S':
                    // allow 100ms as can be 104 characters
                    if ( g_byState == PARSE_COMPLETE )
                    {
                        SendReadings();
                    }
                    break;        
                case 'V':
                    if ( g_byState == PARSE_COMPLETE )
                    {
                        SendVoltageReading();
                    }
                    break;
#endif
                }
                if ( !bReady )
                {
                    if ( GetTickCount() > g_tNextReady )
                    {
                        bReady = 1;
                    }
                }
                g_byState = PARSE_START;
#ifndef RS485_DEVICE
                if ( bReady )
                {
                    SendReady();
                    g_tNextReady = GetTickCount() + 1000;
                }
#endif
            }
        }
    }
    return;
}
#ifdef TIMEOUT
void PerformTimeoutSetAction()
{
    // Turn lights on
    unsigned char n;
    for( n = 1; n <= LED_COUNT; n++)
    {
        SetLED(n, 0xFF);
    }
}
void PerformTimeoutExpiredAction()
{
    // Turn Lights off
    unsigned char n;
    for( n = 1; n <= LED_COUNT; n++)
    {
        SetLED(n, 0);
    }
    // Send 'N'
    char szCMD[5];
    szCMD[0] = '=';
    szCMD[1] = 'N';
    szCMD[2] = '1';
    szCMD[3] = '\r';
    szCMD[4] = '\n';
    SendReady();
    SendBytes(szCMD,5);
}
unsigned long g_SW1Timeout = 0;
unsigned long g_LastTick = 0;
void CheckTimeout()
{
    if ( g_SW1Timeout > 0 )
    {
        unsigned long t = GetTickCount();
        unsigned long diff = t - g_LastTick;
        if ( t < g_LastTick )
        {
            diff = 0xFFFFFFFF - g_LastTick + t + 1;
        }
        g_LastTick = t;
        if ( diff >= g_SW1Timeout )
        {
            g_SW1Timeout = 0;
            PerformTimeoutExpiredAction();
        }
        else
        {
            g_SW1Timeout -= diff;
        }
    }
}
void SetTimeout(unsigned long timeout)
{
    g_SW1Timeout = timeout * TICKS_PER_SECOND;
    g_LastTick = GetTickCount();
    PerformTimeoutSetAction();
}
#endif
#ifdef SWITCH_SENDER
void SendSwitch(BYTE bySwitch, BYTE bAlreadyPressed, BYTE bOtherPressed)
{
    char szCMD[5];
    szCMD[0] = '=';
    szCMD[2] = '1' + bySwitch + LED_OFFSET;
    szCMD[3] = '\r';
    szCMD[4] = '\n';
    if ( bAlreadyPressed )
    {
#ifdef SEND_Z
        if ( bOtherPressed )
        {
            szCMD[1] = 'Z';
        }
        else
#endif
        {
            szCMD[1] = 'D';
        }
    }
    else
    {
#ifdef SW1_OPERATES_LIGHTS
        szCMD[1] = 'Y';
        SetTimeout(TIMEOUT);
#else
        szCMD[1] = 'T';
#endif
    }
    if ( !bAlreadyPressed )
    {
        // TODO - update existing switches to send this first.
        // send ID first to ensure it is set correctly.
        SendReady();
    }
    SendBytes(szCMD,5);
}
#endif
void ReportResetToBus(BYTE byResetReason)
{
    char szCMD[6];
    SendReady();
    szCMD[0] = '!';
    szCMD[1] = 'R';
    szCMD[2] = HexChar(SWITCH_ID >> 4);
    szCMD[3] = HexChar(SWITCH_ID & 0xF);
    szCMD[4] = byResetReason + '0';
    szCMD[5] = 0;
    SendBytes(szCMD,5);
    SendReady();
}
#ifdef SWITCH_SENDER
BYTE GetPort(unsigned nPort)
{
    switch(nPort)
    {
#ifdef SW1_PORT
    case 1:
        return SW1_PORT;
#endif
#ifdef SW2_PORT
    case 2:
        return SW2_PORT;
#endif
#ifdef SW3_PORT
    case 3:
        return SW3_PORT;
#endif
#ifdef SW4_PORT
    case 4:
        return SW4_PORT;
#endif
    default:
        return 0;
    }    
}
typedef struct tagSWITCHSTATE
{
    int nCount;    
    BYTE nPort;                // switch number - 1 on board
    unsigned long tDebounceTick; // Time after which another press counts as a press
    unsigned long tDimTick;      // Time after which continuous press counts as dimming
    unsigned bPressed : 1;     // Indicates if switch is pressed
    unsigned bTick : 1;        // last bit of tick count when last tested
    unsigned bTickRollover : 1;// Indicates tick has rolled over
    unsigned Filler : 5;
}SWITCHSTATE;

#ifdef OLD_SETLEVEL
#define DIM_COUNT 500
#define TOGGLE_COUNT 5000
#define OFF_COUNT 100

void SetLevel(SWITCHSTATE* pSwitch, BYTE bySwitch, SWITCHSTATE* pSwitches)
{
    BYTE byPressed = GetPort(pSwitch->nPort) ? 0 : 1;
    BYTE bOtherPressed = 0;
    BYTE n;
    for( n = 0; n < SWITCH_COUNT; n++ )
    {
        if ( n != bySwitch )
        {
            if ( GetPort(pSwitches[n].nPort))
            {
                bOtherPressed = 1;
            }
        }
    }
    // always do count down.
    if ( pSwitch->nCount == 0 )
    {
        if ( byPressed )
        {
            // TODO - distinguish between a continuous press
            // and debounce - easier if pSwitch is passed.
            // TODO - UDPLogpacket remove reaction to Z command. DONE
            SendSwitch(bySwitch, pSwitch->bPressed, bOtherPressed);
        }
        if ( !byPressed )
        {
            pSwitch->nCount = OFF_COUNT;
        }
        else if ( pSwitch->bPressed )
        {
            pSwitch->nCount = DIM_COUNT;
        }
        else
        {
            pSwitch->nCount = TOGGLE_COUNT;
        }
        pSwitch->bTick = GetTickCount() & 1;
        pSwitch->bPressed = byPressed;
    }
    else if ( pSwitch->nCount > 0 )
    {
        long lTick = GetTickCount();
        if ( (lTick & 1) != pSwitch->bTick )
        {
            pSwitch->nCount--;
            pSwitch->bTick = lTick & 1;
        }
//        if ( !byPressed )
//        {
//            pSwitch->bPressed = 0;
//        }
    }
}
#else
#define DIM_TICKS ((300 * TICKS_PER_SECOND)/1000)
#define DIM_START_TICKS ((1000 * TICKS_PER_SECOND)/1000)
#define TOGGLE_TICKS ((500 * TICKS_PER_SECOND)/1000)
#define OFF_TICKS ((10 * TICKS_PER_SECOND)/1000)
// If tDEbounceTick was set before tick roll over and roll over occurs
// then press will wait until tDebounce tick is reached several hours later.
void SetLevel(SWITCHSTATE* pSwitch, BYTE bySwitch, SWITCHSTATE* pSwitches)
{
    BYTE byPressed = GetPort(pSwitch->nPort) ? 0 : 1;
    BYTE bOtherPressed = 0;
    BYTE n;
    for( n = 0; n < SWITCH_COUNT; n++ )
    {
        if ( n != bySwitch )
        {
            if ( GetPort(pSwitches[n].nPort))
            {
                bOtherPressed = 1;
            }
        }
    }
    unsigned long tTick = GetTickCount();
    if ( pSwitch->bTickRollover )
    {
        pSwitch->bTickRollover = 0;
        pSwitch->tDebounceTick = 0;
        pSwitch->tDimTick = 0;
    }
    if ( byPressed )
    {
        if ( !pSwitch->bPressed )
        {
            if ( tTick > pSwitch->tDebounceTick )
            {   
                SendSwitch(bySwitch, pSwitch->bPressed, 0);
                pSwitch->tDebounceTick = tTick + TOGGLE_TICKS;
                pSwitch->tDimTick = tTick + DIM_START_TICKS;
                pSwitch->bPressed = 1;
            }
        }
        else
        {
            if ( tTick > pSwitch->tDimTick )
            {
                SendSwitch(bySwitch, pSwitch->bPressed, bOtherPressed);
                pSwitch->tDimTick = tTick + DIM_TICKS;
            }
        }
    }
    else
    {
        pSwitch->tDimTick = tTick + DIM_START_TICKS;
        pSwitch->bPressed = 0;
    }
}
#endif
void SetTickRollover(SWITCHSTATE* pSwitch)
{
    int n;
    for( n = 0; n < SWITCH_COUNT; n++ )
    {
        pSwitch[n].bTickRollover = 1;
    }
}
void InitSwitches(SWITCHSTATE* pSwitch)
{
    int n;
    for( n = 0; n < SWITCH_COUNT; n++ )
    {
        pSwitch[n].nPort = n + 1;
    }
}
void ZeroSwitches(SWITCHSTATE* pSwitch)
{
    memset(pSwitch, 0, SWITCH_COUNT * sizeof(SWITCHSTATE));
    InitSwitches(pSwitch);
}
#endif
//
// Main application entry point.
//
void main(void)
{
    static BYTE byResetReason = 0;
    static long LastTimer = 0;
#ifdef SWITCH_SENDER
    static SWITCHSTATE SwitchState[SWITCH_COUNT];
#endif
    
    if ( STKPTRbits.STKUNF )
    {
        byResetReason = 1;
    }
    else if ( STKPTRbits.STKOVF)
    {
        byResetReason = 2;
    }
    else if ( !RCONbits.NOT_RI )
    {
        byResetReason = 7;
        RCONbits.NOT_RI=1;
    }
    else if ( !RCONbits.NOT_POR && !RCONbits.NOT_BOR )
    {
        byResetReason = 4;
        RCONbits.NOT_POR = 1;
        RCONbits.NOT_BOR = 1;
    }
    else if ( !RCONbits.NOT_BOR )
    {
        byResetReason = 3;
        RCONbits.NOT_BOR = 1;
    }
    else if ( !RCONbits.NOT_POR )
    {
        byResetReason = 10;
        RCONbits.NOT_POR = 1;
    }
    else if ( !RCONbits.NOT_PD )
    {
        byResetReason = 5;
    }
    else if ( !RCONbits.NOT_TO )
    {
        byResetReason = 6;
    }
    else if ( STKPTRbits.STKFUL )
    {
        byResetReason = 8;
    }
    else if ( RCONbits.NOT_TO )
    {
        byResetReason = 9;
    }
    // Initialize application specific hardware
    InitializeBoard();
    // Start the timer
//#define NO_INTERRUPTS 0
#ifndef NO_INTERRUPTS
    StartTimer();
#endif
#if !defined(AMPLIFIER) && !defined(ANALOG_AUDIO)
    StartPWM();
#endif
    memset(g_LEDLevels,0,sizeof(g_LEDLevels));
#ifdef SW1_OPERATES_LIGHTS
    SetTimeout(2);
    PerformTimeoutExpiredAction();
#endif
#if ANALOG_AUDIO
    InitAnalogAudio();
#endif
    //.
    //  Output reason for reset - eventually
    //  we need to formalise an error packet going
    //  to RS232 but this will do for now
    //
    // enable the watchdog timer in case it gets stuck in a loop
    WDTCONbits.SWDTEN = 1;
#ifndef NO_INTERRUPTS
    LastTimer = GetTickCount() + 1;
    // at power on reset allow a small delay
    // Report reset to bus
    while(GetTickCount() <= LastTimer );
#endif
    ReportResetToBus(byResetReason);
    //
    // Initialise switch and program group states
    //
#ifdef SWITCH_SENDER
    ZeroSwitches(SwitchState);
#endif
#if defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
    StartADC();
#endif
    // enter main service loop
    int nBits = 0;
    int nLastClock = 0;
    while(1)
    {
        BYTE n;
        BYTE ch = 0;
        // clear the WDT at the start of loop to stop it firing
#asm
        CLRWDT
#endasm
#ifdef SWITCH_SENDER
        if ( GetTickCount() < LastTimer )
        {
            SetTickRollover(SwitchState);
        }
        LastTimer = GetTickCount();
#endif
        // Parse and act on any input
#ifdef NO_INTERRUPTS
        SetVolume(0, 0x3F);
#else
        ReceiveService();
#endif  
        // Transfer anything from slave board
#ifdef FORWARD_UART2
        TransferRC2toTX1();
#endif
#ifdef SWITCH_SENDER
        // Respond to switches
        for( n = 0; n < SWITCH_COUNT; n++ )
        {
            SetLevel(SwitchState + n, n, SwitchState);
        }
#endif
#if defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
        ServiceADC();
#endif
        // Check timeout if defined
#if defined(TIMEOUT)
        CheckTimeout();  
#endif
        // Set LEDs

#if !defined(NO_MANUAL_PWM) && !defined(AMPLIFIER) && !defined(ANALOG_AUDIO))
#ifndef MANUAL_PWM_USE_TIMER
        DoManualPWM();
#endif
#endif
    }
}
/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void InitializeBoard(void)
{	
    // disable watchdog timer
    WDTCONbits.SWDTEN = 0;
//#if defined(__18CXX)
    #if GetSystemClock() > 4000000
	// Enable 4x/5x/96MHz PLL on PIC18F87J10, PIC18F97J60, PIC18F87J50, etc.
#ifdef ENABLE_PLL
    OSCTUNE = 0x40;
#endif
    #endif
    OSCCON2 = 0;
    // set clock to chosen speed (should be 4mhz here by default anyway)
    OSCCONbits.IDLEN = 0;  // sleep on sleep instruction
    OSCCONbits.IRCF = IRCF_VALUE;   // 4 Mhz
    OSCCONbits.OSTS = 0;
    OSCCONbits.SCS = 3;
    // wait for clock to stabilise
    while( !OSCCONbits.HFIOFS);
    // Set up analog features of PORTA
    // Ensure all pins associated with UART are digital
    RX_ANSEL = 0;
    TX_ANSEL = 0;
    RX_TRIS = 1; // Configure as input

    RX2_ANSEL = 0;
    TX2_ANSEL = 0;
    RX2_TRIS = 1;
    // Enable internal PORTB pull-ups
#ifndef ANALOG_AUDIO
    INTCON2bits.RBPU = 0;
#endif
#ifdef SWITCH_SENDER
    SW1_WPUB = 1;
#ifdef SW2_WPUB
    SW2_WPUB = 1;
#endif
#ifdef SW3_WPUB
    SW3_WPUB = 1;
#endif
#ifdef SW4_WPUB
    SW4_WPUB = 1;
#endif
#endif
    // Configure USART
    TXSTA = 0;
    RCSTA = 0;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN = 1;
    BAUDCON = 0;
    BAUDCONbits.BRG16 = 1;
    BAUDCONbits.CKTXP=0;
    TXSTAbits.BRGH = 1;
#ifdef SYSTEM_CLOCK_4
#warning Running at 4Mhz
    SPBRG = 51;
    SPBRGH = 0;
#elif SYSTEM_CLOCK_16
#warning Running at 16Mhz
    SPBRG = 207;
    SPBRGH = 0;
#else
#warning Running at 48Mhz
    SPBRG=0x70; //(112)
    SPBRGH=2;
#endif
 // only enable transmit if not using pin to configure other device
    TXSTA2 = 0;
    RCSTA2 = 0;
    BAUDCON2 = 0;
    BAUDCON2bits.BRG16 = 1;
    RCSTA2bits.SPEN = 1;
    RCSTA2bits.CREN = 1;
    BAUDCON2bits.BRG16 = 1;
    BAUDCON2bits.CKTXP=0;
    TXSTA2bits.BRGH = 1;
#ifdef SYSTEM_CLOCK_4
// EPIR uses a rate of 9600 rather than 19200
    SPBRG2 = 51;
    SPBRGH2 = 0;
#elif SYSTEM_CLOCK_16
#warning Running at 16Mhz
    SPBRG2 = 207;
    SPBRGH2 = 0;
#else
#warning Running at 48Mhz
    SPBRG2=0x70; //(112)
    SPBRGH2=2;
#endif
#ifdef INPUT_SPI
    // Configure SSP1 as slave
    ANSELCbits.ANSC3 = 0;
    ANSELCbits.ANSC4 = 0;
    ANSELCbits.ANSC5 = 0;
    ANSELAbits.ANSA5 = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;
    TRISAbits.TRISA5 = 1;
    SSP1STAT = 0;
    SSP1CON1 = 0;
    SSP1CON2 = 0;
    SSP1CON3 = 0;
    SSP1STATbits.SMP = 0;
    SSP1STATbits.CKE = 1;
    SSP1CON1bits.WCOL = 0;
    SSP1CON1bits.CKP = 0;
    SSP1CON1bits.SSPM = 4;  // SPI Slave - !SS1 enabled
//    SSP1CON1bits.SSPM1 = 5;  // SPI Slave - !SS1 disabled
    SSP1CON3bits.BOEN = 1;
    SSP1CON1bits.SSPEN1 = 1;
#endif
// Configure switched
#ifdef SWITCH_SENDER
    SW1_ANSEL = 0;
    SW1_TRIS = 1;
#ifdef SW2_ANSEL
    SW2_ANSEL = 0;
    SW2_TRIS = 1;
#endif
#ifdef SW3_ANSEL
    SW3_ANSEL = 0;
    SW3_TRIS = 1;
#endif
#ifdef SW4_ANSEL
    SW4_ANSEL = 0;
    SW4_TRIS = 1;
#endif
#endif
#ifdef LED1_TRIS
    LED1_ANSEL=0;
    LED1_TRIS=0;
    LED1_LAT=0;
#endif
#ifdef LED2_TRIS
    LED2_ANSEL=0;
    LED2_TRIS=0;
    LED2_LAT=0;
#endif
    PMD1bits.CCP5MD= 0;
#ifdef LED3_TRIS
    LED3_ANSEL=0;
    LED3_TRIS=0;
    LED3_LAT=0;
#endif
#ifdef LED4_TRIS
    LED4_ANSEL=0;
    LED4_TRIS=0;
    LED4_LAT=0;
#endif
#ifdef LED5_TRIS
    LED5_ANSEL = 0;
    LED5_TRIS = 0;
    LED5_LAT=0;
#endif
#ifdef LED6_TRIS
    LED6_ANSEL = 0;
    LED6_TRIS = 0;
    LED6_LAT=0;
#endif
#ifdef LED7_TRIS
    LED7_ANSEL = 0;
    LED7_TRIS = 0;
    LED7_LAT=0;
#endif
#ifdef LED8_TRIS
    LED8_ANSEL = 0;
    LED8_TRIS = 0;
    LED8_LAT=0;
#endif
#ifdef LED9_TRIS
    LED9_ANSEL = 0;
    LED9_TRIS = 0;
    LED9_LAT=0;
#endif
#ifdef LED10_TRIS
    LED10_ANSEL = 0;
    LED10_TRIS = 0;
    LED10_LAT=0;
#endif
#ifdef LED11_TRIS
    LED11_ANSEL = 0;
    LED11_TRIS = 0;
    LED11_LAT=0;
#endif
#ifdef LED12_TRIS
    LED12_ANSEL = 0;
    LED12_TRIS = 0;
    LED12_LAT=0;
#endif
#ifdef LED13_TRIS
    LED13_ANSEL = 0;
    LED13_TRIS = 0;
    LED13_LAT=0;
#endif
#ifdef LED14_TRIS
    LED14_ANSEL = 0;
    LED14_TRIS = 0;
    LED14_LAT=0;
#endif
#ifdef LED15_TRIS
    LED15_ANSEL = 0;
    LED15_TRIS = 0;
    LED15_LAT=0;
#endif
#ifdef LED16_TRIS
    LED16_ANSEL = 0;
    LED16_TRIS = 0;
    LED16_LAT=0;
#endif
    
#if defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER)
    ADC1_TRIS = 1;
    ADC1_ANSEL = 1;
    ADC2_TRIS = 1;
    ADC2_ANSEL = 1;
    ADC3_TRIS = 1;
    ADC3_ANSEL = 1;
    ADC4_TRIS = 1;
    ADC4_ANSEL = 1;
#ifdef ADC5_TRIS
    ADC5_TRIS = 1;
    ADC5_ANSEL = 1;
#endif
#ifdef ADC6_TRIS
    ADC6_TRIS = 1;
    ADC6_ANSEL = 1;
#endif
#ifdef ADC7_TRIS
    ADC7_TRIS = 1;
    ADC7_ANSEL = 1;
#endif
#ifdef ADC8_TRIS
    ADC8_TRIS = 1;
    ADC8_ANSEL = 1;
#endif
#if defined(ADC9_TRIS) && defined(ADC9_ANSEL)
    ADC9_TRIS = 1;
    ADC9_ANSEL = 1;
#endif
    SetupADC();
#endif

// DI is an output
    DI_ANSEL = 0;
    DI_TRIS = 0;
    DI_IO = 0;
    // Enable Interrupts
    PIE1 = 0;
    PIE2 = 0;
    PIE3 = 0;
    PIE4 = 0;
    PIE5 = 0;
    INTCONbits.INT0IE = 0;
    INTCONbits.RBIE = 0;
    INTCON3bits.INT2IE = 0;
    INTCON3bits.INT1IE = 0;
    RCONbits.IPEN = 1;		// Enable interrupt priorities

    PIE1bits.RC1IE = 1;
    IPR1bits.RC1IP = 1;
    PIE1bits.TX1IE = 0;
    IPR1bits.TX1IP = 0;
    // Enable interrupts for UART2 as well
    PIE3bits.RC2IE = 1;
    IPR3bits.RC2IP = 1;
    PIE3bits.TX2IE = 0;
    IPR3bits.TX2IP = 0;
#ifdef INPUT_SPI
    PIE1bits.SSP1IE = 1;
    IPR1bits.SSP1IP = 1;
#endif
#ifndef NO_INTERRUPTS
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
#else
    INTCONbits.GIEH = 0;
    INTCONbits.GIEL = 0;
#endif
}
