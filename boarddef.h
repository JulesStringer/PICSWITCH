// ID Written at physical location of switch
#define SWITCH_ID 222
// Z for test with UART
// Y for test with SPI

// Versions
// 13  Version 1_3 of lightcontroller - no longer used
// 14  Version 1_4 of lightcontroller - no longer used
// 15  50X25 board with RS422 interface and 10 pin connector
// 16  50X25 board with RS422 interface and 20 pin connector
// 17  50X25 board with SPI and RS485 , 20 pin - used for UART2
// 18  50X25 board with straight 2 6 pin UART headers and 20 pin connector
// 19  As 16 but LED1 is on A5 rather than A7
// 20  Power Switch has 16 LED outputs no switch inputs based on 18
// 21  Garden light controller has 4 LED and 1 Switch connected to radio control based on 18
// 22  Analog Audio controller board
//
#if SWITCH_ID == 1
// bottom of stairs
#define VERSION 16
#endif

#if SWITCH_ID == 41
// utility
#define VERSION 16
#endif
#if SWITCH_ID == 48
// dining door
#define VERSION 16
#define INPUT_UART2 0
#endif
#if SWITCH_ID == 49
#define FORWARD_UART2
// leanto
#endif
#if SWITCH_ID == 67
#define VERSION 16
// cloakroom
#endif

#if SWITCH_ID == 69
#define FORWARD_UART2 0
// cloadkroom
#define VERSION 16
#endif
#if SWITCH_ID == 70
#define INPUT_UART2
// cloakroom
#define VERSION 16
#endif

#if SWITCH_ID == 81
#define VERSION 16
// top of stairs
#endif

#if SWITCH_ID == 88
#define VERSION 16
// bathroom
#endif

#if SWITCH_ID == 98
#define VERSION 16
// test
#endif
#if SWITCH_ID == 99
#define INPUT_SPI 0
// test
#endif


#if ((SWITCH_ID >= 101 && SWITCH_ID <= 109) || (SWITCH_ID >= 161 && SWITCH_ID <= 186 ))

#if SWITCH_ID == 165
#define LED_OFFSET 8
#define VERSION 17
#define PROCESS_BATTERY 0
#define CHARGER_PIN 6
#define CHARGER_PROGRAM 20
#define INPUT_UART2 0
#elif (SWITCH_ID == 178 )
#define VERSION 16
#elif (SWITCH_ID == 173 )
#define VERSION 17
#elif (SWITCH_ID == 184 )
#define VERSION 21
#else
#define VERSION 18
#endif
#define LIGHT_CONTROLLER 0  // measure current on ADC pins rather than switching
#if (VERSION < 15)
#define RS485_DEVICE 0
#if (SWITCH_ID >= 101 && SWITCH_ID <= 109 )
#define MASTER_DEVICE 210
#elif (SWITCH_ID == 161 )
#define MASTER_DEVICE 210
#elif (SWITCH_ID >= 162 && SWITCH_ID <= 165)
#define MASTER_DEVICE 212
#elif (SWITCH_ID >= 166 && SWITCH_ID <= 177)
#define MASTER_DEVICE 211
#elif (SWITCH_ID >= 185 && SWITCH_ID <= 186)
#define MASTER_DEVICE 214
#endif
#endif
#if SWITCH_ID == 101
#define FORWARD_UART2
#endif
#if SWITCH_ID == 102
#define FORWARD_UART2
#endif
#if SWITCH_ID == 103
#define FORWARD_UART2
#endif
#if SWITCH_ID == 104
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 105
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 106
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 107
#define FORWARD_UART2
#endif
#if SWITCH_ID == 108
#define FORWARD_UART2
#endif
#if SWITCH_ID == 109
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 161
#define LED_OFFSET 8
#define PROCESS_BATTERY 0
#define CHARGER_PIN 7
#define CHARGER_PROGRAM 21
#endif
#if SWITCH_ID == 162
#define FORWARD_UART2
#define VERSION 19
#endif
#if SWITCH_ID == 163
#define INPUT_UART2 0
#define VERSION 17
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 164
#define FORWARD_UART2
#define VERSION 19
#endif
#if SWITCH_ID == 166   //F
#define FORWARD_UART2
#endif
#if SWITCH_ID == 167   //G
#define LED_OFFSET 8
#define FORWARD_UART2
#endif
#if SWITCH_ID == 168   //H
//#define FORWARD_UART2
#define LED_OFFSET 16
#endif
#if SWITCH_ID == 169   //I
#define FORWARD_UART2
#endif
#if SWITCH_ID == 170   // J
#define LED_OFFSET 8
#define FORWARD_UART2
#endif
#if SWITCH_ID == 171   // K
#define LED_OFFSET 16
//#define FORWARD_UART2
#endif
#if SWITCH_ID == 173   // M
#define LED_OFFSET 8
//#define INPUT_UART2
#endif
#if SWITCH_ID == 174   // N
#define FORWARD_UART2
//#define LED_OFFSET 8
#endif
#if SWITCH_ID == 175   // O
#define LED_OFFSET 16
#endif
#if SWITCH_ID == 176   // P
#define FORWARD_UART2
#define LED_OFFSET 8
#endif
#if SWITCH_ID == 177  // Q
#define FORWARD_UART2
#define PROCESS_BATTERY 0
#define CHARGER_PIN 8
#define CHARGER_PROGRAM 22
#endif
                       // R 178 under stairs defined above
#if SWITCH_ID == 184   // X driveway lights controller
#define LED_OFFSET 0
#define TIMEOUT 60     // timeout for lights
#define SW1_OPERATES_LIGHTS 1
#define SWITCH_SENDER 0
#define VERSION 21
#define OLD_SETLEVEL 0
#endif
#if SWITCH_ID == 185   // Y
#define VERSION 19
#define LED_OFFSET 8
#define FORWARD_UART2
#endif
#if SWITCH_ID == 186   // Z
#define VERSION 19
#define LED_OFFSET 0
#define FORWARD_UART2
//#define PROCESS_BATTERY 0
//#define CHARGER_PIN 6
//#define CHARGER_PROGRAM 20
#endif
#endif

#if SWITCH_ID == 128
#define VERSION 16
#endif

#if SWITCH_ID == 129
#define VERSION 16
#define FORWARD_UART2
#endif

#if SWITCH_ID == 130
#define VERSION 16
#endif

#if SWITCH_ID == 132
#define VERSION 16
#endif

#if SWITCH_ID == 150
// movement sensor back garage
#define MASTER_DEVICE 212
#endif
#if SWITCH_ID == 188
#define VERSION 16
#define INPUT_UART2 0
#endif

#if SWITCH_ID == 190
#define VERSION 16
#endif

#if SWITCH_ID == 220 || SWITCH_ID == 221
#define VERSION 20
#define POWER_SWITCH 0
//#define LED_COUNT 8
//#define AMPLIFIER 0
//#define AMP_DELAY 100
#endif

#if SWITCH_ID == 222
#define VERSION 22
#define ANALOG_AUDIO 1
#endif
#ifndef VERSION
#define VERSION 15
#endif

#ifndef LED_OFFSET
#ifdef INPUT_UART2
#ifdef LIGHT_CONTROLLER
#define LED_OFFSET 8
#else
#define LED_OFFSET 4
#endif
#else
#define LED_OFFSET 0
#endif
#endif

#if !defined(LIGHT_CONTROLLER) && !defined(AMPLIFIER) && !defined(POWER_SWITCH) && !defined(ANALOG_AUDIO)
#define SWITCH_SENDER 0
#endif

#ifdef RS485_DEVICE
#ifndef MASTER_DEVICE
// default to test device
#define MASTER_DEVICE 214
#endif
#else
#define MASTER_DEVICE 0
#endif