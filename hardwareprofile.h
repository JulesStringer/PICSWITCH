#include <GenericTypeDefs.h>
#include <xc.h>

#define SYSTEM_CLOCK_16 1

#include "boarddef.h"
#ifdef LIGHT_CONTROLLER
#if VERSION == 13
#include "HardwareProfile LIGHT_CONTROLLER_V1_3.h"
#endif
#if VERSION == 14
#include "HardwareProfile LIGHT_CONTROLLER_V1_4.h"
#endif
#endif
#if VERSION == 15
#ifdef REVERSE_SWITCHES
#include "HardwareProfile LIGHT_SWITCH_V1_5_Reversed.h"
#else
#include "HardwareProfile LIGHT_SWITCH_V1_5.h"
#endif
#elif (VERSION == 16 || VERSION == 19)
#include "HardwareProfile LIGHT_SWITCH_V1_6.h"
#elif VERSION == 17
#include "HardwareProfile LIGHT_CONTROLLER_V1_7.h"
#elif VERSION == 18
#include "HardwareProfile LIGHT_CONTROLLER_V1_8.h"
#elif VERSION == 20
#include "HardwareProfile POWER_SWITCH_1_8.h"
#elif VERSION == 21
#include "HardwareProfile COMBINED_V1_8.h"
#elif VERSION == 22
#include "HardwareProfile_ANALOG_AUDIO.h"
#endif
