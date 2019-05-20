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

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"
#include "pin.h"
#include "gpio.h"
#include "flc.h"
#include "timer.h"

// WolkConnect C includes
#include "wolk_connector.h"
#include "wolk_utils.h"

// App Includes
#include "device_status.h"
#include "smartconfig.h"
#include "tmp006drv.h"
#include "flc_api.h"

// Common interface includes
#include "gpio_if.h"
#include "udma_if.h"
#include "common.h"
#include "i2c_if.h"
#include "timer_if.h"

#ifndef NOTERM
#include "uart_if.h"
#endif

// Custom headers
#include "pinmux.h"
#include "boot_info.h"
#include "init_parser.h"

#define APPLICATION_NAME        "WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad"
#define APPLICATION_VERSION     "1.1.1"
#define CONFIG_FILE             "/sys/config.txt"
#define USER_IMAGE_1            "/sys/mcuimg2.bin"
#define USER_IMAGE_2            "/sys/mcuimg3.bin"


#define BUF_SIZE            2048
#define TCP_PACKET_COUNT    1000
#define SL_STOP_TIMEOUT     200

#define BLINK_INTERVAL_MS   1000
#define SENSOR_INTERVAL_S   5

#define DATE                20;          // Day of month (DD format) range 1-31
#define MONTH               05;          // Month (MM format) in the range of 1-12
#define YEAR                2019;        // Year (YYYY format)
#define HOUR                14;          // Hours in the range of 0-23
#define MINUTE              30;          // Minutes in the range of 0-59
#define SECOND              00;          // Seconds in the range of  0-59

//#define CA_CERT "ca.der"
#define CA_CERT "cawritten.der"
//#define CA_CERT "ca_cert.der"
#define CA_CERT_VERSION "ca_version.txt"
#define CA_CERTIFICATE_VERSION 1

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    FILE_ALREADY_EXIST = -0x7D0,
    FILE_CLOSE_ERROR = FILE_ALREADY_EXIST - 1,
    FILE_NOT_MATCHED = FILE_CLOSE_ERROR - 1,
    FILE_OPEN_READ_FAILED = FILE_NOT_MATCHED - 1,
    FILE_OPEN_WRITE_FAILED = FILE_OPEN_READ_FAILED -1,
    FILE_READ_FAILED = FILE_OPEN_WRITE_FAILED - 1,
    FILE_WRITE_FAILED = FILE_READ_FAILED - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


#endif /* MAIN_H_ */
