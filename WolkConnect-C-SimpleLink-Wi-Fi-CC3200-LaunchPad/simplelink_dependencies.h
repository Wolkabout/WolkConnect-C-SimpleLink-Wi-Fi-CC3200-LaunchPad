/*
 * simplelink_dependencies.h
 *
 *  Created on: 20.05.2019.
 *      Author: srdjan.stankovic
 */

#ifndef SIMPLELINK_DEPENDENCIES_H_
#define SIMPLELINK_DEPENDENCIES_H_

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"

#include "flc.h"
#include "hw_types.h"

#ifndef NOTERM
#include "uart_if.h"
#endif

#include "common.h"

#define APPLICATION_NAME        "WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad"
#define APPLICATION_VERSION     "1.1.1"
#define CONFIG_FILE             "/sys/config.txt"

#define CA_CERT "cawritten.der"
#define CA_CERT_VERSION "ca_version.txt"
#define CA_CERTIFICATE_VERSION 1

#define DATE                20;          // Day of month (DD format) range 1-31
#define MONTH               05;          // Month (MM format) in the range of 1-12
#define YEAR                2019;        // Year (YYYY format)
#define HOUR                14;          // Hours in the range of 0-23
#define MINUTE              30;          // Minutes in the range of 0-59
#define SECOND              00;          // Seconds in the range of  0-59


#define BUF_SIZE            2048
#define TCP_PACKET_COUNT    1000
#define SL_STOP_TIMEOUT     200

#define BLINK_INTERVAL_MS   1000
#define SENSOR_INTERVAL_S   5

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


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static int iSockID;
unsigned long  g_ulStatus;                              //SimpleLink Status
static unsigned long  g_ulGatewayIP = 0;                //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1];      //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX];      //Connection BSSID
volatile unsigned long  g_ulPacketCount;

static unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];
unsigned char gaucCmpBuf[128];

char* wifi_network_name;
unsigned char wifi_network_security_type;
char* wifi_network_password;

unsigned long ip_addr;

static const char *hostname = "mqtt-verification2.wolkabout.com";
static bool secure = 0;
static unsigned short port_num = 1883;

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

char* device_key;
char* device_password;

bool led_on;
bool led_enabled;
bool led_mode;
bool has_sensor_reading;
sBootInfo_t boot_info;
//****************************************************************************
//                      FUNCTION PROTOTYPES
//****************************************************************************
int8_t initializeCC3200();
static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();
static long ConfigureSimpleLinkToDefaultState();
static void LEDBlinkyRoutine();
static signed char set_current_device_time(void);
static bool update_ca_cert(void);

int SslTcpClient(unsigned short usPort);
long WlanConnect();

#endif /* SIMPLELINK_DEPENDENCIES_H_ */
