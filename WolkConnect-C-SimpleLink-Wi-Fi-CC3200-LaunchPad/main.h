#ifndef MAIN_H_
#define MAIN_H_

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"

#include "flc.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "timer.h"
#include "nonos.h"
#include "fs.h"
#include "flc.h"
#include "boot_info.h"

#include "common.h"
#include "tmp006drv.h"

// WolkConnect C includes
#include "wolk_connector.h"
#include "wolk_utils.h"

#include "simplelink_dependencies.h"
#include "wolkconnect_dependencies.h"

static volatile bool keep_running = true;


//*****************************************************************************
//                   WOLK VARIABLES -- START
//*****************************************************************************
static const char* actuator_references[] = {"led"};
static const uint32_t num_actuator_references = 1;
static const char* sensor_reference = "temp";
static uint8_t persistence_storage[8*1024];
static wolk_ctx_t wolk;

//****************************************************************************
//                            FIRMWARE VARIABLES
//****************************************************************************

static const long max_firmware_size = 100*1024;
//const int16_t firmware_chunk_size = 512;
sBootInfo_t boot_info;


#endif /* MAIN_H_ */
