/*
 * simplelink_dependencies.c
 *
 *  Created on: 20.05.2019.
 *      Author: srdjan.stankovic
 */

// driverlib includes
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"
#include "pin.h"
#include "gpio.h"
#include "device.h"

// App Includes
#include "device_status.h"
#include "smartconfig.h"
#include "tmp006drv.h"
#include "flc_api.h"

// Common interface includes
#include "gpio_if.h"
#include "udma_if.h"
#include "i2c_if.h"
#include "timer_if.h"

// Custom headers
#include "pinmux.h"
#include "boot_info.h"
#include "init_parser.h"

#include "simplelink_dependencies.h"

#include "flc.h"
#include "timer.h"
#include "string.h"

/*!
 * \brief Function reboots MCU
 * \return None
 */
static void RebootMCU()
{
  // Configure hibernate RTC wakeup
  PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);
  // Delay loop
  MAP_UtilsDelay(8000000);
  // Set wake up time
  PRCMHibernateIntervalSet(330);
//  wolk_disconnect(&wolk); //TODO
  // Request hibernate
  PRCMHibernateEnter();
  // Control should never reach here
  while(1);
}


//****************************************************************************
//                 LOCAL FUNCTIONS IMPLEMENTATIONS
//****************************************************************************
int8_t initializeCC3200() {
    int lRetVal = -1;
    // Board Initialization
    BoardInit();

    // Initialize the uDMA
    UDMAInit();

    // Configure the pinmux settings for the peripherals exercised
    PinMuxConfig();

    // Configuring UART
    InitTerm();

    // Display banner
    DisplayBanner(APPLICATION_NAME);
    InitializeAppVariables();

    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
        {
            UART_PRINT("Failed to configure the device in its default state \n\r");
        }
        return lRetVal;
    }
    UART_PRINT("Device is configured in default state \n\r");

    // Asumption is that the device is configured in station mode already
    // and it is in its default state
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    //      boot_info.ucActiveImg = IMG_ACT_FACTORY;
    ////
    //      boot_info.ulImgStatus = IMG_STATUS_NOTEST;
    ////
    //      WriteBootInfo(&boot_info);

    ReadBootInfo(&boot_info);
    switch(boot_info.ucActiveImg) {
    case IMG_ACT_FACTORY:
        UART_PRINT("Device booted FACTORY image \n\r");
        break;
    case IMG_ACT_USER1:
        UART_PRINT("Device booted USER1 image \n\r");
        break;
    case IMG_ACT_USER2:
        UART_PRINT("Device booted USER2 image \n\r");
        break;
    }

    // Testin init file
    if (init_file_parse(CONFIG_FILE) != 0)
    {
        UART_PRINT("Config file couldn't be parsed \n\r");
        return lRetVal;
    }

    lRetVal = set_current_device_time();
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to set current device time \n\r");

        return lRetVal;
    }

    //update cert
    lRetVal = update_ca_cert();
    if( lRetVal < 0)
        ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Config file values read: \n\r");
    device_key = parser_get_device_key();
    device_password = parser_get_device_password();
    wifi_network_name = parser_get_wifi_network_name();
    wifi_network_security_type = parser_get_wifi_network_security_type();
    wifi_network_password = parser_get_wifi_network_password();
    UART_PRINT("SSID name: %s\n\r", parser_get_wifi_network_name());
    UART_PRINT("SSID security type: %d\n\r", parser_get_wifi_network_security_type());
    UART_PRINT("SSID password: %s\n\r", parser_get_wifi_network_password());
    UART_PRINT("Device key: %s\n\r", parser_get_device_key());
    UART_PRINT("Device password: %s\n\r", parser_get_device_password());

    UART_PRINT("Device started as STATION \n\r");
    UART_PRINT("Connecting to AP: %s ... \n\r", wifi_network_name);
    // Connecting to WLAN AP - Set with static parameters read from config file
    // After this call we will be connected and have IP address
    lRetVal = WlanConnect();
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to AP failed \n\r");
        return lRetVal;
    }

    UART_PRINT("Connected to AP: %s \n\r", wifi_network_name);
    UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r",
               SL_IPV4_BYTE(g_ulIpAddr,3),
               SL_IPV4_BYTE(g_ulIpAddr,2),
               SL_IPV4_BYTE(g_ulIpAddr,1),
               SL_IPV4_BYTE(g_ulIpAddr,0));

    //ping to check if exist
    UART_PRINT("Ping 8.8.8.8 \n\r");
    SlPingReport_t report;
    SlPingStartCommand_t pingCommand;

    pingCommand.Ip = SL_IPV4_VAL(8,8,8,8);     // destination IP address is 10.1.1.200
    pingCommand.PingSize = 50;                   // size of ping, in bytes
    pingCommand.PingIntervalTime = 80;           // delay between pings, in milliseconds
    pingCommand.PingRequestTimeout = 500;        // timeout for every ping in milliseconds
    pingCommand.TotalNumberOfAttempts = 3;       // max number of ping requests. 0 - forever
    pingCommand.Flags = 0;                        // report only when finished

    if (sl_NetAppPingStart( &pingCommand, SL_AF_INET, &report, NULL ) != SUCCESS )
        ASSERT_ON_ERROR(-1);
    UART_PRINT("------------------\n\rPing report\n\r------------------\n\r" \
             "PacketsSent: %lu\n\r" \
             "PacketsReceived: %lu\n\r" \
             "MinRoundTime: %d\n\r" \
             "MaxRoundTime: %d\n\r" \
             "AvgRoundTime: %d\n\r" \
             "TestTime: %lu\n\r------------------\n\r" \
             , report.PacketsSent, report.PacketsReceived, report.MinRoundTime, report.MaxRoundTime, report.AvgRoundTime, report.TestTime);

    lRetVal = SslTcpClient((unsigned short)port_num);
    if(lRetVal < 0)
    {
        UART_PRINT("TCP Client failed\n\r");
        return lRetVal;
    }

    // Configure LEDs
    GPIO_IF_LedConfigure(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);

    // I2C Init for Temperature sensor
    lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);
    if(lRetVal < 0)
    {
        UART_PRINT("Couldn't open I2C connection to sensors! \n\r");
        ERR_PRINT(lRetVal);
        return lRetVal;
    }

    //Init Temprature Sensor
    lRetVal = TMP006DrvOpen();
    if(lRetVal < 0)
    {
        UART_PRINT("Couldn't find temperature sensor on I2C! \n\r");
        ERR_PRINT(lRetVal);
        return lRetVal;
    }

    // Setup timer interrupt
    //    80000000=SYS_CLK Convert to ms -> ((SYS_CLK/1000) * (ms))
    PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA3);
    TimerConfigure(TIMERA3_BASE, TIMER_CFG_A_PERIODIC);
    IntPrioritySet(INT_TIMERA3A, INT_PRIORITY_LVL_7);
    TimerIntRegister(TIMERA3_BASE, TIMER_A, LEDBlinkyRoutine);
    TimerIntEnable(TIMERA3_BASE, TIMER_TIMA_TIMEOUT);
    TimerLoadSet(TIMERA3_BASE, TIMER_A, (80000000/1000) * BLINK_INTERVAL_MS);

    return 0;
}

/*!
 * \brief Application startup display on UART
 * \param[in]  char* AppName
 * \return None
 */
static void DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t ****************************************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t ****************************************************************************\n\r");
    Report("\n\n\n\r");
}

/*!
 * \brief Board Initialization & Configuration
 * \param None
 * \return None
 */
static void BoardInit(void)
{
    // In case of TI-RTOS vector table is initialize by OS itself
#ifndef USE_TIRTOS
  // Set vector table base
#if defined(ccs) || defined (gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/*!
 * \brief This function initializes the application variables
 * \param[in]  None
 * \return None
 */
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_ulPacketCount = TCP_PACKET_COUNT;
    g_ulStatus = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

/*!
 * \brief This function puts the device in its default state. It:
 *           - Set the mode to STATION
 *           - Configures connection policy to Auto and AutoSmartConfig
 *           - Deletes all the stored profiles
 *           - Enables DHCP
 *           - Disables Scan policy
 *           - Sets Tx power to maximum
 *           - Sets power policy to normal
 *           - Unregister mDNS services
 *           - Remove all filters
 *
 * \param  none
 * \return  On success, zero is returned. On error, negative is returned
 */
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while (!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while (IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

/*!
 * \brief Routine which is called on timer interrupt
 * \return None
 */
static void LEDBlinkyRoutine()
{
    static long long no_interrupt = 0;

    Timer_IF_InterruptClear(TIMERA3_BASE);
    no_interrupt++;
    if(led_enabled)
    {
        if(led_mode)
        {
            if (led_on) {
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                led_on = false;
            } else
            {
                GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                led_on = true;
            }
        }
        else
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            led_on = false;
        }

    } else
    {
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }

    if(no_interrupt >= 30) {
       has_sensor_reading = true;
       no_interrupt = 0;
    }
}

/*!
 * \brief Opening a TCP client side socket and sending data
 * \details This function opens a TCP socket and tries to connect to a Server IP_ADDR
 *          waiting on port PORT_NUM. If the socket connection is successful then the
 *          function will send 1000 TCP packets to the server.
 * \param[in]  port number on which the server will be listening on
 * \return 0 on success, -1 on Error.
 */

int SslTcpClient(unsigned short usPort)
{
    UART_PRINT("Entering SslTcpClient\n\r");

    int             iCounter;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iStatus;
    int16_t set_socket_option_result = 0;

    // filling the buffer
    for (iCounter = 0; iCounter < BUF_SIZE; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    unsigned long ulIpAddr = 0;
    char host_name[128];
    strncpy(host_name, hostname, strlen(hostname));
    long lRetVal = sl_NetAppDnsGetHostByName((signed char*)host_name, strlen(hostname), &ulIpAddr, SL_AF_INET);
    if(lRetVal < 0 || ulIpAddr == 0)
    {
        UART_PRINT("Error to get IP address from DNS\n\r");
        ASSERT_ON_ERROR(lRetVal);
    }
    //temporarly
    ip_addr = ulIpAddr;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, (secure ? SL_SEC_SOCKET : 0));
    if (iSockID < 0)
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    //SSL setup
    if(secure)
    {
        SlSockNonblocking_t socket_non_blocking;
        socket_non_blocking.NonblockingEnabled = 1;
        set_socket_option_result = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &socket_non_blocking, sizeof(SlSockNonblocking_t));
        if(set_socket_option_result < 0)
        {
            UART_PRINT("Unable to set blocking mode\n\r");
            ASSERT_ON_ERROR(-1);
        }

        SlSockSecureMethod method;
        method.secureMethod = SL_SO_SEC_METHOD_TLSV1_2;//SL_SO_SEC_METHOD_SSLv3_TLSV1_2;
        set_socket_option_result = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(SlSockSecureMethod));
        if(set_socket_option_result < 0)
        {
            UART_PRINT("Unable to set socket security methods\n\r");
            ASSERT_ON_ERROR(set_socket_option_result);
        }

        SlSockSecureMask mask;
        mask.secureMask = SL_SEC_MASK_SECURE_DEFAULT;//SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA;
        set_socket_option_result = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(SlSockSecureMask));
        if(set_socket_option_result < 0)
        {
            UART_PRINT("Unable to set socket security mask\n\r");
            ASSERT_ON_ERROR(set_socket_option_result);
        }

        UART_PRINT("\n\r--------------------------------------------------------\n\r--------------------------------------------------------\n\r" \
                   "Reading %s file from file system...\n\r", CA_CERT);

        unsigned long token = 0;
        _u8 ReadBuffer[CERT_NUM_CHAR];
        signed long ca_version_file = 0;
        int32_t bytes_read = 0;

        int32_t file_opened = sl_FsOpen(CA_CERT, FS_MODE_OPEN_READ, &token, &ca_version_file);
        if(file_opened != 0)
        {
            UART_PRINT("File does not exist! \n\r");
            ASSERT_ON_ERROR(file_opened);
        }
        else
        {
            bytes_read = sl_FsRead(ca_version_file, 0, (_u8 *)ReadBuffer, CERT_NUM_CHAR);
            if (bytes_read < 0)
            {
                UART_PRINT("Error reading %s file from file system \n\r", CA_CERT);
                ASSERT_ON_ERROR(bytes_read);
            }
            else
            {
                UART_PRINT("Number of read bytes from file is: %d\r\n\r\n", bytes_read);

                for(int i=0; i<CERT_NUM_CHAR; i++){
                    UART_PRINT("%c", ReadBuffer[i]);
                }

                UART_PRINT("\n\r--------------------------------------------------------\n\r--------------------------------------------------------\n\r\n\r");
            }
        }

        set_socket_option_result = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_FILES_CA_FILE_NAME, (unsigned char *)CA_CERT, strlen((const unsigned char *)CA_CERT));
        if( set_socket_option_result < 0 )
        {
            UART_PRINT("Unable to set socket certificate");
            ASSERT_ON_ERROR(set_socket_option_result);
        }
    }


    //check time
    _i8 retVal = -1;
    SlDateTime_t dateTime= {0};
    _u8 configLen = sizeof(SlDateTime_t);
    _u8 configOpt = SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME;
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION,&configOpt, &configLen,(_u8 *)(&dateTime));
    if( retVal < 0)
    {
        ASSERT_ON_ERROR(retVal);
    }
    UART_PRINT("Read Date and Time is D/M/Y - H:M:S : %d/%d/%d - %.2d:%.2d:%.2d \n\r", dateTime.sl_tm_day, dateTime.sl_tm_mon, dateTime.sl_tm_year, dateTime.sl_tm_hour, dateTime.sl_tm_min, dateTime.sl_tm_sec);


    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short) usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int) ulIpAddr);
    iAddrSize = sizeof(SlSockAddrIn_t);
    UART_PRINT("\n\rServer IP is: 0x%x\n\rServer port is: %u\n\r", ulIpAddr, usPort);

    uint8_t retries = 0;
    while( ((iStatus=sl_Connect(iSockID, (SlSockAddr_t*)&sAddr, iAddrSize))!=0) && (retries++<200) )
    {
        if(iStatus == SL_EALREADY) // wait to be opened
        {
            UART_PRINT("SL_EALREADY, continue trying...\n\r");
            MAP_UtilsDelay(8000000);
        }
        else if(iStatus < 0) // error
        {
            ASSERT_ON_ERROR(iStatus);
        }
    }

    UART_PRINT("%s - Connected\n\r", hostname);

    return SUCCESS;
}

/*!
 * \brief Connecting to a WLAN Accesspoint
 * \details This function connects to the required AP (wifi_network_name) with Security
 *          parameters specified in te form of macros at the top of this file
 * \param[in]  None
 * \return Status value
 * \warining If the WLAN connection fails or we don't aquire an IP address, it
 *           will be stuck in this function forever.
 */
long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)wifi_network_password;
    secParams.KeyLen = strlen(wifi_network_password);
    secParams.Type = wifi_network_security_type;

    lRetVal = sl_WlanConnect((signed char*)wifi_network_name, strlen(wifi_network_name), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* Wait */
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    return SUCCESS;
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

/*!
 * \brief The Function Handles WLAN Events
 * \param[in]  pWlanEvent - Pointer to WLAN Event Info
 * \return None
 */
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }

    switch (pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
                        " BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if (SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                            "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

/*!
 * \brief This function handles network events such as IP acquisition, IP
 *        leased, IP released etc.
 * \param[in]  pNetAppEvent - Pointer to NetApp Event Info
 * \return None
 */
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if (!pNetAppEvent)
    {
        return;
    }

    switch (pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_ulIpAddr = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                        "Gateway=%d.%d.%d.%d\n\r",
                            SL_IPV4_BYTE(g_ulIpAddr,3),
                            SL_IPV4_BYTE(g_ulIpAddr,2),
                            SL_IPV4_BYTE(g_ulIpAddr,1),
                            SL_IPV4_BYTE(g_ulIpAddr,0),
                            SL_IPV4_BYTE(g_ulGatewayIP,3),
                            SL_IPV4_BYTE(g_ulGatewayIP,2),
                            SL_IPV4_BYTE(g_ulGatewayIP,1),
                            SL_IPV4_BYTE(g_ulGatewayIP,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}

/*!
 * \brief This function handles HTTP server events
 * \param[in]  pServerEvent - Contains the relevant event information
 * \param[in]  pServerResponse - Should be filled by the user with the
 *               relevant response information
 * \return None
 */
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

/*!
 * \brief This function handles General Events
 * \param[in]  pDevEvent - Pointer to General Event Info
 * \return None
 */
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n\r",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

/*!
 * \brief This function handles socket events indication
 * \param[in]      pSock - Pointer to Socket Event Info
 * \return None
 */
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if (!pSock)
    {
        return;
    }

    // This application doesn't work w/ socket - Events are not expected
    // Close firmware file and reboot MCU
//    if (firmware_file != NULL)
//    {
//        sl_FsClose(firmware_file, 0, 0, 0);
//    }
    switch (pSock->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch (pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n\r",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    RebootMCU();
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n\r",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                    RebootMCU();
                    break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n\r",pSock->Event);
            RebootMCU();
            break;
    }
}

/*!
 * \brief This function set current time in the device
 * \param[in] None
 * \return None
 */
static signed char set_current_device_time(void)
{
     _i8 retVal = -1;
     SlDateTime_t dateTime= {0};

     dateTime.sl_tm_day =   (_u32)DATE;
     dateTime.sl_tm_mon =   (_u32)MONTH;
     dateTime.sl_tm_year =  (_u32)YEAR;
     dateTime.sl_tm_hour =  (_u32)HOUR;
     dateTime.sl_tm_min =   (_u32)MINUTE;
     dateTime.sl_tm_sec =   (_u32)SECOND;

     retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION, SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME, sizeof(SlDateTime_t), (_u8 *)(&dateTime));
     if( retVal < 0)
     {
         ASSERT_ON_ERROR(retVal);
     }

     _u8 configLen = sizeof(SlDateTime_t);
     _u8 configOpt = SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME;
     retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION,&configOpt, &configLen,(_u8 *)(&dateTime));
     if( retVal < 0)
     {
         ASSERT_ON_ERROR(retVal);
     }
     UART_PRINT("Read Date and Time is D/M/Y - H:M:S : %d/%d/%d - %.2d:%.2d:%.2d \n\r", dateTime.sl_tm_day, dateTime.sl_tm_mon, dateTime.sl_tm_year, dateTime.sl_tm_hour, dateTime.sl_tm_min, dateTime.sl_tm_sec);

     return retVal;
}

/*!
 * \brief This function write ca.der file onto device file system with name
 * define with CA_CERT
 *
 * \param[in] None
 *
 * \return None
 */
bool update_ca_cert(void)
{
    UART_PRINT("Updating ca certificate\r\n");

    unsigned long token = 0;
    int32_t file_opened = 0;

    //ca.crt file
    const uint8_t cert[] = {0x30, 0x82, 0x03, 0xF9, 0x30, 0x82, 0x02, 0xE1, 0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x09, 0x00, 0xBC, 0xE2, 0x55, 0xD4, 0x2D, 0x7F, 0xF7, 0x1D, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x30, 0x81, 0x91, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x52, 0x53, 0x31, 0x0F, 0x30, 0x0D, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0C, 0x06, 0x53, 0x65, 0x72, 0x62, 0x69, 0x61, 0x31, 0x11, 0x30, 0x0F, 0x06, 0x03, 0x55, 0x04, 0x07, 0x0C, 0x08, 0x4E, 0x6F, 0x76, 0x69, 0x20, 0x53, 0x61, 0x64, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x09, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x31, 0x0C, 0x30, 0x0A, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x0C, 0x03, 0x52, 0x26, 0x44, 0x31, 0x16, 0x30, 0x14, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x0D, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x31, 0x24, 0x30, 0x22, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x09, 0x01, 0x16, 0x15, 0x73, 0x75, 0x70, 0x70, 0x6F, 0x72, 0x74, 0x40, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x30, 0x20, 0x17, 0x0D, 0x31, 0x35, 0x31, 0x32, 0x31, 0x30, 0x31, 0x32, 0x33, 0x39, 0x31, 0x30, 0x5A, 0x18, 0x0F, 0x32, 0x30, 0x36, 0x35, 0x31, 0x31, 0x32, 0x37, 0x31, 0x32, 0x33, 0x39, 0x31, 0x30, 0x5A, 0x30, 0x81, 0x91, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x52, 0x53, 0x31, 0x0F, 0x30, 0x0D, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0C, 0x06, 0x53, 0x65, 0x72, 0x62, 0x69, 0x61, 0x31, 0x11, 0x30, 0x0F, 0x06, 0x03, 0x55, 0x04, 0x07, 0x0C, 0x08, 0x4E, 0x6F, 0x76, 0x69, 0x20, 0x53, 0x61, 0x64, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x09, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x31, 0x0C, 0x30, 0x0A, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x0C, 0x03, 0x52, 0x26, 0x44, 0x31, 0x16, 0x30, 0x14, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x0D, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x31, 0x24, 0x30, 0x22, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x09, 0x01, 0x16, 0x15, 0x73, 0x75, 0x70, 0x70, 0x6F, 0x72, 0x74, 0x40, 0x77, 0x6F, 0x6C, 0x6B, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82, 0x01, 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xAE, 0xD0, 0xB6, 0x27, 0xF2, 0x7C, 0x54, 0xA8, 0x57, 0x32, 0x4B, 0x5A, 0xE8, 0xEA, 0x30, 0xA5, 0x62, 0x80, 0x17, 0x48, 0x77, 0x09, 0xAC, 0x03, 0x52, 0x54, 0xD8, 0x7B, 0x0F, 0x27, 0x9E, 0x57, 0xA0, 0xA7, 0xE7, 0x0C, 0xD6, 0xD6, 0x8A, 0xC8, 0xDB, 0xAF, 0x51, 0xDE, 0x41, 0x66, 0x7B, 0xA2, 0x8F, 0xB8, 0x2F, 0x1D, 0x71, 0xA4, 0xE5, 0x00, 0x27, 0x75, 0x16, 0x7D, 0xFD, 0xF2, 0x59, 0xC4, 0x0C, 0x7D, 0xC6, 0x68, 0xBA, 0xD9, 0x9E, 0x37, 0xEF, 0x6D, 0x65, 0x9F, 0x1F, 0xF7, 0x61, 0x18, 0x1F, 0x6C, 0x4D, 0x75, 0x64, 0x5C, 0x2D, 0x3F, 0x1F, 0x32, 0x36, 0x0D, 0xBA, 0x2E, 0x3C, 0x6C, 0xC1, 0x71, 0x38, 0x7E, 0x25, 0x12, 0x20, 0x59, 0xF2, 0x78, 0xF7, 0x07, 0xBC, 0x47, 0xE7, 0x39, 0x83, 0xEC, 0x4C, 0xF9, 0x7F, 0xB8, 0xAB, 0x43, 0xE8, 0x3B, 0x09, 0x28, 0x9D, 0x63, 0xB9, 0x66, 0x47, 0xF5, 0x46, 0x49, 0xE7, 0x16, 0xDE, 0x6B, 0x31, 0xCA, 0x49, 0x5D, 0xD4, 0x5D, 0xD9, 0xF3, 0xF2, 0x70, 0x8C, 0x9B, 0xFD, 0x42, 0x91, 0xAE, 0xA9, 0xF8, 0xF6, 0xBC, 0xA2, 0x9F, 0x78, 0xFD, 0xF7, 0xD6, 0x7B, 0xAE, 0xB7, 0x36, 0x25, 0x25, 0xEF, 0x59, 0x11, 0x1F, 0x73, 0x70, 0x8F, 0xC1, 0x26, 0xDC, 0x50, 0x1B, 0xCA, 0x06, 0xC3, 0x4F, 0xDD, 0x43, 0x8C, 0xB7, 0xF0, 0xA3, 0x0D, 0x6E, 0x39, 0x4A, 0x59, 0x99, 0xE8, 0x01, 0x97, 0x36, 0x21, 0x8E, 0x0A, 0xAE, 0x21, 0x93, 0x50, 0x29, 0x54, 0x97, 0x7F, 0xA4, 0x73, 0x1D, 0xA0, 0x31, 0xD7, 0x55, 0x8C, 0xAF, 0x96, 0xF9, 0xBC, 0x65, 0x05, 0xD1, 0x81, 0x5C, 0xC7, 0xA0, 0x2E, 0xA7, 0x0D, 0x29, 0x95, 0x0F, 0x10, 0xBB, 0x69, 0xEA, 0x71, 0x48, 0xDA, 0x56, 0x62, 0x6C, 0xF1, 0x26, 0x8C, 0x9E, 0xC4, 0xEB, 0x85, 0xF5, 0x49, 0x83, 0x02, 0x03, 0x01, 0x00, 0x01, 0xA3, 0x50, 0x30, 0x4E, 0x30, 0x1D, 0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x0B, 0x93, 0x9C, 0x4B, 0x1B, 0xB1, 0xE4, 0xCF, 0xB7, 0x0C, 0xFB, 0x1A, 0x37, 0x54, 0x4A, 0x28, 0xCC, 0x7D, 0x31, 0x9F, 0x30, 0x1F, 0x06, 0x03, 0x55, 0x1D, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x0B, 0x93, 0x9C, 0x4B, 0x1B, 0xB1, 0xE4, 0xCF, 0xB7, 0x0C, 0xFB, 0x1A, 0x37, 0x54, 0x4A, 0x28, 0xCC, 0x7D, 0x31, 0x9F, 0x30, 0x0C, 0x06, 0x03, 0x55, 0x1D, 0x13, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xFF, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x03, 0x82, 0x01, 0x01, 0x00, 0x9C, 0x47, 0xE8, 0x72, 0xB8, 0xEC, 0x2D, 0xC7, 0xF6, 0x74, 0x27, 0x80, 0xE4, 0x6E, 0xEC, 0x9E, 0xA4, 0x11, 0xFD, 0x5D, 0x8A, 0x32, 0x6E, 0x95, 0x51, 0x1D, 0x28, 0x19, 0xA3, 0x98, 0x2B, 0xA0, 0x03, 0x2C, 0x3A, 0x07, 0x9B, 0xE1, 0xDB, 0x09, 0x89, 0x4C, 0x3F, 0x93, 0xFA, 0x34, 0x62, 0x62, 0xC4, 0x10, 0xA7, 0xBE, 0xF6, 0xD3, 0xFD, 0xC2, 0x22, 0xCA, 0x29, 0xDE, 0x19, 0x03, 0x53, 0x01, 0xFF, 0x19, 0xF9, 0x73, 0x5C, 0x37, 0xA2, 0x0E, 0xCB, 0x20, 0x7A, 0xA7, 0xC3, 0x6A, 0x01, 0x3C, 0xE7, 0x7E, 0xBF, 0x88, 0x95, 0x64, 0xC0, 0x6B, 0xFA, 0xA6, 0x2D, 0xFA, 0x71, 0xF1, 0x42, 0x18, 0x4B, 0x93, 0x4F, 0xA4, 0x6D, 0x28, 0x52, 0x94, 0x0F, 0xB2, 0xE5, 0x0B, 0x4C, 0x1E, 0x07, 0xE2, 0xF3, 0xAC, 0xA3, 0x05, 0xF1, 0x6A, 0xB9, 0x14, 0x82, 0xF9, 0x4E, 0xA1, 0x7C, 0x37, 0xA0, 0xBE, 0x63, 0x56, 0xF5, 0x99, 0xE9, 0xC8, 0x6F, 0x14, 0xBE, 0x96, 0x92, 0x40, 0x5A, 0xA7, 0xCE, 0x08, 0x2B, 0xBF, 0x23, 0xE2, 0xBA, 0x9A, 0x6D, 0x35, 0x95, 0x44, 0x0F, 0x97, 0xE7, 0x92, 0x8D, 0xE6, 0x79, 0x5A, 0x5E, 0xDA, 0x7E, 0xEA, 0x95, 0xB8, 0x5D, 0x84, 0x37, 0x30, 0x20, 0x5F, 0x58, 0x57, 0x6C, 0xB5, 0x83, 0xAF, 0xCF, 0xDA, 0x4D, 0xF3, 0x94, 0xAF, 0xE4, 0x68, 0x09, 0x0E, 0xEC, 0xA5, 0x83, 0xF1, 0x87, 0x19, 0x78, 0x3C, 0x88, 0xB1, 0x3B, 0x63, 0xF0, 0xA4, 0xB7, 0xFE, 0xEB, 0x8D, 0x47, 0xED, 0x35, 0x5F, 0x01, 0x1F, 0xDE, 0x66, 0x90, 0x0F, 0x5F, 0xEE, 0xF6, 0xBF, 0x54, 0x08, 0x17, 0x0A, 0xDF, 0x58, 0xD6, 0x2F, 0xA9, 0x00, 0x49, 0xBD, 0x46, 0x6E, 0xA9, 0xD2, 0x18, 0xBE, 0x85, 0xAD, 0x9F, 0x38, 0x8F, 0x28, 0x6B, 0xB6, 0xB2, 0x65, 0xD1, 0xE8, 0xFE, 0x28, 0x6B, 0xA5};
    uint32_t cert_size = sizeof(cert);

    signed long ca_file = 0;
    file_opened = sl_FsOpen(CA_CERT, FS_MODE_OPEN_CREATE(cert_size, _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE), &token, &ca_file);
    if(file_opened < 0)
    {
        UART_PRINT("Error creating ca file %d\r\n", (int16_t)file_opened);
        return FAILURE;
    }

    if(sl_FsWrite(ca_file, 0, cert, cert_size) != cert_size)
    {
        UART_PRINT("Cert not written succefully\r\n");
        return FAILURE;
    }

    UART_PRINT("Cert written succefully\r\n");

    int16_t ca_file_closed = sl_FsClose(ca_file, 0, 0, 0);
    if(ca_file_closed < 0)
    {
        UART_PRINT("Error closing ca file\r\n");
        return FAILURE;
    }

    return SUCCESS;
}
