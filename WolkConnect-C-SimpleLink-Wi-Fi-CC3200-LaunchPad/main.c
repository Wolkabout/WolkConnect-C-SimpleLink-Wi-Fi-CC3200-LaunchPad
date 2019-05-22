/*
 * Copyright 2018 WolkAbout Technology s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * NOTE:
 *      Error on SSL connection exist. Error code is -456, which means "error secure level bad CA file"
 */

#include "main.h"


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
        UART_PRINT("Socket error, trying to reopen it\n\r Error code is: %d\n\r", n);
        if (sl_Close(iSockID) != SUCCESS)
            ASSERT_ON_ERROR(SOCKET_CLOSE_ERROR);

        int lRetVal = SslTcpClient((unsigned short)port_num);
        // Try to reconnect to wlan
        if (lRetVal < 0)
        {
            UART_PRINT("Trying to reconnect to wlan \n\r");
            // Blocking function
            if (WlanConnect() != SUCCESS)
                ASSERT_ON_ERROR(-1);

            UART_PRINT("Reconnected to wlan \n\r");
            int lRetVal = SslTcpClient((unsigned short)port_num);
            if (lRetVal < 0)
                ASSERT_ON_ERROR(lRetVal);

            if (wolk_connect(&wolk) != W_FALSE)
            {
                UART_PRINT("Wolk client - Error connecting to server \n\r");
                return FAILURE;
            }
        }
//        return FAILURE;
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
        int lRetVal = SslTcpClient((unsigned short)port_num);
        // Try to reconnect to wlan
        if (lRetVal < 0)
        {
            UART_PRINT("Trying to reconnect to wlan \n\r");
            // Blocking function
            WlanConnect();
            UART_PRINT("Reconnected to wlan \n\r");
            SslTcpClient((unsigned short)port_num);
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
              configuration_handler, configuration_provider,
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
    UART_PRINT("Wolk client - try publishing...\n\r");
    wolk_publish(&wolk);
    UART_PRINT("Wolk client - Published\n\r");

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
        wolk_process(&wolk, 5);
        _SlNonOsMainLoopTask();
    }

    UART_PRINT("Wolk client - Disconnecting\n\r");
    wolk_disconnect(&wolk);

    UART_PRINT("Exiting Application ...\n\r");
    // power off the Network processor
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }
}
