#include "main.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static int iSockID;
volatile unsigned long  g_ulStatus = 0;                 //SimpleLink Status
unsigned long  g_ulGatewayIP = 0;                       //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1];      //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX];      //Connection BSSID
volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];
unsigned char gaucCmpBuf[128];
// Connection variables
char* wifi_network_name;
unsigned char wifi_network_security_type;
char* wifi_network_password;
char* device_key;
char* device_password;
unsigned long ip_addr = 0x34d510e3;         // Demo IP 52.213.16.227 https://api-demo.wolkabout.com
unsigned short port_num = 1883;             // Demo port 1883

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                   WOLK VARIABLES -- START
//*****************************************************************************
static const char* actuator_references[] = {"led"};
static const uint32_t num_actuator_references = 1;
static const char* sensor_reference = "temp";
static uint8_t persistence_storage[8*1024];
static wolk_ctx_t wolk;
static volatile bool keep_running = true;
static bool led_on = false;
static bool led_enabled = false;
static bool has_sensor_reading = false;
//****************************************************************************
//                            FIRMWARE VARIABLES
//****************************************************************************
static long firmware_file;
static size_t firmware_file_size = 0;
static unsigned long firmware_file_offset = 0;
static const long max_firmware_size = 100*1024;
static const int16_t firmware_chunk_size = 512;
static const char* firmware_version_file = "firmware_version";
static const int16_t firmware_version_file_size = 20;
static int total_chunk_count = 0;
static int packet_no = 0;
static sBootInfo_t boot_info;
//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int8_t initializeCC3200();
static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();
static long ConfigureSimpleLinkToDefaultState();
static void RebootMCU();
static int BsdTcpClient(unsigned short usPort);
static long WlanConnect();
void LEDBlinkyRoutine();

//****************************************************************************
//                      WOLK CONNECTIVITY STARTS HERE
//****************************************************************************
static void actuation_handler(const char* reference, const char* value)
{
    printf("Actuation handler - Reference: %s Value: %s\n\r", reference, value);
    if (strcmp(reference, "led") == 0)
    {
        if(strcmp(value, "false") == 0)
        {
            led_enabled = false;
        } else if (strcmp(value, "true") == 0)
        {
            led_enabled = true;
        }
    }
}

static actuator_status_t actuator_status_provider(const char* reference)
{
    printf("Actuator status provider - Reference: %s\n\r", reference);

    actuator_status_t actuator_status;
    actuator_status_init(&actuator_status, "", ACTUATOR_STATE_ERROR);

    if (strcmp(reference, "led") == 0)
    {
        if(led_enabled)
        {
            actuator_status_init(&actuator_status, "true", ACTUATOR_STATE_READY);
        } else
        {
            actuator_status_init(&actuator_status, "false", ACTUATOR_STATE_READY);
        }
    }
    return actuator_status;
}

static int send_buffer(unsigned char* buffer, unsigned int len)
{
    int n;
    SlSockAddrIn_t  sAddr;
    int iAddrSize;

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)port_num);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)ip_addr);

    iAddrSize = sizeof(SlSockAddrIn_t);

    n = sl_SendTo(iSockID, buffer, len, 0, (SlSockAddr_t *)&sAddr ,iAddrSize);
    if (n < 0)
    {
        // Try to reopen socket
        UART_PRINT("Socket error, trying to reopen it \n\r");
        sl_Close(iSockID);
        int lRetVal = BsdTcpClient((unsigned short)port_num);
        // Try to reconnect to wlan
        if (lRetVal < 0)
        {
            UART_PRINT("Trying to reconnect to wlan \n\r");
            // Blocking function
            WlanConnect();
            UART_PRINT("Reconnected to wlan \n\r");
            BsdTcpClient((unsigned short)port_num);
            if (wolk_connect(&wolk) != W_FALSE)
            {
                UART_PRINT("Wolk client - Error connecting to server \n\r");
                return -1;
            }
        }
        return -1;
    }

    return n;
}

static int receive_buffer(unsigned char* buffer, unsigned int max_bytes)
{
    int n;

    n = sl_Recv(iSockID, buffer, max_bytes, 0);
    if (n < 0)
    {
        // Try to reopen socket
        UART_PRINT("Socket error, trying to reopen it \n\r");
        sl_Close(iSockID);
        int lRetVal = BsdTcpClient((unsigned short)port_num);
        // Try to reconnect to wlan
        if (lRetVal < 0)
        {
            UART_PRINT("Trying to reconnect to wlan \n\r");
            // Blocking function
            WlanConnect();
            UART_PRINT("Reconnected to wlan \n\r");
            BsdTcpClient((unsigned short)port_num);
            if (wolk_connect(&wolk) != W_FALSE)
            {
                UART_PRINT("Wolk client - Error connecting to server \n\r");
                return -1;
            }
        }
        return -1;
    }

    return n;
}

//****************************************************************************
//                            FIRMWARE HANDLERS
//****************************************************************************
static bool firmware_update_start(const char* file_name, size_t file_size)
{
    long lRetVal = -1;
    const unsigned char* image_name;

    if(boot_info.ucActiveImg == IMG_ACT_FACTORY || boot_info.ucActiveImg == IMG_ACT_USER2)
    {
        image_name = USER_IMAGE_1;
        sl_FsDel(USER_IMAGE_1, 0);
    } else if (boot_info.ucActiveImg == IMG_ACT_USER1) {
        image_name = USER_IMAGE_2;
        lRetVal = sl_FsDel(USER_IMAGE_2, 0);
    } else {
        UART_PRINT("Error reading active image, cannot update firmware!\n\r");
        return false;
    }

    UART_PRINT("Starting firmware update. File name %s. File size %lld\n\r", file_name, (unsigned long long int)file_size);
    firmware_file_offset = 0;
    firmware_file_size = file_size;
    packet_no = 0;
    total_chunk_count = (double)firmware_file_size / firmware_chunk_size;

    lRetVal = sl_FsOpen((unsigned char*) image_name,
                        FS_MODE_OPEN_CREATE(firmware_file_size, \
                         _FS_FILE_PUBLIC_WRITE|_FS_FILE_PUBLIC_READ),
                         NULL,
                         &firmware_file);
    if(lRetVal < 0)
    {
        // File may already be created
        lRetVal = sl_FsClose(firmware_file, 0, 0, 0);
        return false;
    }
    else
    {
        // Close the user file
        lRetVal = sl_FsClose(firmware_file, 0, 0, 0);
        if (SL_RET_CODE_OK != lRetVal)
        {
            return false;
        }
    }

    // Open a user file for writing
    lRetVal = sl_FsOpen((unsigned char *)image_name,
                        FS_MODE_OPEN_WRITE|FS_MODE_OPEN_READ,
                        NULL,
                        &firmware_file);
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(firmware_file, 0, 0, 0);
        return false;
    }

    return true;
}

static bool firmware_chunk_write(uint8_t* data, size_t data_size)
{
    long written_bytes = 0;
    int try_write = 3;
    int i = 0;

    for(i = 0; i < try_write; i++)
    {
        UART_PRINT("Firmware update chunk write %d/%d \n\r", packet_no, total_chunk_count);
        written_bytes = sl_FsWrite(firmware_file,
                                   firmware_file_offset,
                                   data,
                                   data_size);

        UART_PRINT("Written %d \n\r", written_bytes);
        if (written_bytes >= 0)
        {
            break;
        }
    }
    if (written_bytes < 0)
    {
        UART_PRINT("Writing firmware update chunk failed! \n\r");
        UART_PRINT("Firmware file offset %ld\n\r", firmware_file_offset);
        written_bytes = sl_FsClose(firmware_file, 0, 0, 0);
        return false;
    }

    packet_no++;
    firmware_file_offset += written_bytes;
    return true;
}


static size_t firmware_chunk_read(size_t index, uint8_t* data, size_t data_size)
{
    long read_bytes = 0;

    UART_PRINT("Firmware update chunk read \n\r");
    if ((index + 1) * data_size > firmware_file_size)
    {
        data_size = firmware_file_size % data_size;
    }
    read_bytes = sl_FsRead(firmware_file,
                           (unsigned int)((long)index * (long)data_size),
                           data,
                           data_size);

    return read_bytes;
}

static void firmware_update_abort(void)
{
    UART_PRINT("Aborting firmware update! \n\r");
    firmware_file_offset = 0;
    packet_no = 0;
    if (firmware_file != NULL)
    {
        sl_FsClose(firmware_file, 0, 0, 0);
    }
}

static void firmware_update_finalize(void)
{
    sBootInfo_t boot_info_temp;

    firmware_file_offset = 0;
    UART_PRINT("Finalizing firmware update! \n\r");
    if (firmware_file != NULL) {
        sl_FsClose(firmware_file, 0, 0, 0);
    }

    if(boot_info.ucActiveImg == IMG_ACT_FACTORY || boot_info.ucActiveImg == IMG_ACT_USER2)
    {
        boot_info_temp.ucActiveImg = IMG_ACT_USER1;
        UART_PRINT("Switching to USER1 image! \n\r");
    } else {
        boot_info_temp.ucActiveImg = IMG_ACT_USER2;
        UART_PRINT("Switching to USER2 image! \n\r");
    }

    boot_info_temp.ulImgStatus = IMG_STATUS_NOTEST;
    WriteBootInfo(&boot_info_temp);

    RebootMCU();
}

static bool firmware_update_persist_firmware_version(const char* version)
{
    int lRetVal;
    long firmware_version_file_handler;

    // Open or create file
    lRetVal = sl_FsOpen((unsigned char*) firmware_version_file,
                        FS_MODE_OPEN_CREATE(firmware_version_file_size, _FS_FILE_PUBLIC_WRITE|_FS_FILE_PUBLIC_READ),
                        NULL,
                        &firmware_version_file_handler);
    if(lRetVal < 0)
    {
        lRetVal = sl_FsDel((unsigned char*)firmware_version_file, 0);
    }
    // File may already be created
    sl_FsClose(firmware_version_file_handler, 0, 0, 0);

    // Open a user file for writing
    lRetVal = sl_FsOpen((unsigned char *)firmware_version_file,
                        FS_MODE_OPEN_WRITE,
                        NULL,
                        &firmware_version_file_handler);
    if(lRetVal < 0)
    {
        sl_FsClose(firmware_version_file_handler, 0, 0, 0);
        return false;
    }
    // Write version to file
    lRetVal = sl_FsWrite(firmware_version_file_handler,
                        (unsigned int)0,
                        (unsigned char *)version, strlen(version));
    if(lRetVal < 0)
    {
        sl_FsClose(firmware_version_file_handler, 0, 0, 0);
        return false;
    }
    sl_FsClose(firmware_version_file_handler, 0, 0, 0);
    return true;
}

static bool firmware_update_unpersist_firmware_version(char* version, size_t version_size)
{
    long firmware_version_handler = 0;
    long lRetVal = 0;

    // Open file for reading
    lRetVal = sl_FsOpen((unsigned char*) firmware_version_file,
                        FS_MODE_OPEN_READ,
                        NULL,
                        &firmware_version_handler);
    // File does not exist
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(firmware_version_handler, 0, 0, 0);
        return false;
    }
    // Read file
    lRetVal = sl_FsRead(firmware_version_handler,
                        0,
                        (unsigned char*)version,
                        version_size);
    // Read unsuccessful
    if(lRetVal < 0)
    {
        sl_FsClose(firmware_version_handler, 0, 0, 0);
        return false;
    }

    // Read successful
    sl_FsClose(firmware_version_handler, 0, 0, 0);
    // Delete file
    lRetVal = sl_FsDel((const unsigned char*)firmware_version_file, 0);
    if(lRetVal < 0)
    {
        return false;
    }

    UART_PRINT("Unpersist version success! \n\r");
    return true;
}


//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
int main()
{
    long lRetVal = -1;

    lRetVal = initializeCC3200();
    if (lRetVal < 0)
    {
        LOOP_FOREVER();
    }

    // Wolk connectivity starts here
    if (wolk_init(&wolk,
              send_buffer, receive_buffer,
              actuation_handler, actuator_status_provider,
              device_key, device_password, PROTOCOL_JSON_SINGLE, actuator_references, num_actuator_references) != W_FALSE)
    {
        UART_PRINT("Error initializing WolkConnect-C \n\r");
        return 1;
    }

    if (wolk_init_in_memory_persistence(&wolk, persistence_storage, sizeof(persistence_storage), false) != W_FALSE)
    {
        UART_PRINT("Error initializing in-memory persistence \n\r");
        return 1;
    }

    if (wolk_init_firmware_update(&wolk, APPLICATION_VERSION, max_firmware_size,
                                  firmware_chunk_size,
                                  firmware_update_start,
                                  firmware_chunk_write,
                                  firmware_chunk_read,
                                  firmware_update_abort,
                                  firmware_update_finalize,
                                  firmware_update_persist_firmware_version,
                                  firmware_update_unpersist_firmware_version,
                                  NULL /* start url */,
                                  NULL /* url is done*/) != W_FALSE)
    {
        UART_PRINT("Error initializing firmware update \n\r");
        return 1;
    }

    UART_PRINT("Wolk client - Connecting to server \n\r");
    if (wolk_connect(&wolk) != W_FALSE)
    {
        UART_PRINT("Wolk client - Error connecting to server \n\r");
        return -1;
    }
    UART_PRINT("Wolk client - Connected to server \n\r");

    // Get initial temp
    float initial_temp = 0;
    TMP006DrvGetTemp(&initial_temp);
    initial_temp = (initial_temp - 32) * 5 / 9;

    wolk_add_numeric_sensor_reading(&wolk, sensor_reference, initial_temp, 0);
    wolk_publish(&wolk);

    // Enable timer
    TimerEnable(TIMERA3_BASE, TIMER_A);

    while (keep_running)
    {
        if (has_sensor_reading) {
            has_sensor_reading = false;

            float fCurrentTempC = 0;
            // Get temp from sensor
            TMP006DrvGetTemp(&fCurrentTempC);
            fCurrentTempC = (fCurrentTempC - 32) * 5 / 9;

            wolk_add_numeric_sensor_reading(&wolk, sensor_reference, fCurrentTempC, 0);
            wolk_publish(&wolk);
            UART_PRINT("Sensor reading %.2f \n\r", fCurrentTempC);
        }
        wolk_process(&wolk);
        _SlNonOsMainLoopTask();
    }

    printf("Wolk client - Disconnecting\n\r");
    wolk_disconnect(&wolk);

    UART_PRINT("Exiting Application ...\n\r");
    // power off the Network processor
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }
}

//****************************************************************************
//                 LOCAL FUNCTIONS IMPLEMENTATIONS
//****************************************************************************

static int8_t initializeCC3200() {
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
        printf("Config file couldn't be parsed \n\r");
        UART_PRINT("Config file couldn't be parsed \n\r");
        return lRetVal;
    }

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

    lRetVal = BsdTcpClient((unsigned short)port_num);
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
 * \brief Function reboots MCU
 * \return None
 */
void RebootMCU()
{
  // Configure hibernate RTC wakeup
  PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);
  // Delay loop
  MAP_UtilsDelay(8000000);
  // Set wake up time
  PRCMHibernateIntervalSet(330);
  wolk_disconnect(&wolk);
  // Request hibernate
  PRCMHibernateEnter();
  // Control should never reach here
  while(1);
}
/*!
 * \brief Routine which is called on timer interrupt
 * \return None
 */
void LEDBlinkyRoutine()
{
    static long long no_interrupt = 0;

    Timer_IF_InterruptClear(TIMERA3_BASE);
    no_interrupt++;
    if(led_enabled)
    {
        if (led_on) {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            led_on = false;
        } else
        {
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            led_on = true;
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

int BsdTcpClient(unsigned short usPort)
{
    int             iCounter;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iStatus;

    // filling the buffer
    for (iCounter = 0; iCounter < BUF_SIZE; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short) usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int) ip_addr);

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
    if (iSockID < 0)
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if (iStatus < 0)
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

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
static long WlanConnect()
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
    if (firmware_file != NULL)
    {
        sl_FsClose(firmware_file, 0, 0, 0);
    }
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
