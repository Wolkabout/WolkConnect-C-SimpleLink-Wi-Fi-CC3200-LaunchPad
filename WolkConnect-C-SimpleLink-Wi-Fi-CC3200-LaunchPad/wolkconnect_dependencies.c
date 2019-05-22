/*
 * wolkconnect_dependencies.c
 *
 *  Created on: 22 May 2019
 *      Author: srdjan.stankovic
 */
#include "fs.h"
#include "stdlib.h"
#include "common.h"
#include "boot_info.h"

#include "uart_if.h"

// WolkConnect C includes
#include "wolk_connector.h"
#include "wolk_utils.h"

#include "wolkconnect_dependencies.h"
#include "simplelink_dependencies.h"



//****************************************************************************
//                      WOLK CONNECTIVITY STARTS HERE
//****************************************************************************
void actuation_handler(const char* reference, const char* value)
{
    UART_PRINT("Actuation handler - Reference: %s Value: %s\n\r", reference, value);
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

actuator_status_t actuator_status_provider(const char* reference)
{
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
    UART_PRINT("Actuator status provider - Reference: %s | Actuator Status: %s\n\r", reference, actuator_status.value);

    return actuator_status;
}

void configuration_handler(char (*reference)[CONFIGURATION_REFERENCE_SIZE],
                                  char (*value)[CONFIGURATION_VALUE_SIZE],
                                  size_t num_configuration_items)
{
    UART_PRINT("Configuration Handler - Reference: %s | Value: %s\n\r", reference[0], value[0]);

    if(!strcmp("led_mode", reference[0]))
    {
        UART_PRINT("Received Reference is: %s\n\r", reference[0]);
        strcmp("true", value[0])==0 ? (led_mode=true) : (led_mode=false);
    }
}

size_t configuration_provider(char (*reference)[CONFIGURATION_REFERENCE_SIZE],
                                     char (*value)[CONFIGURATION_VALUE_SIZE],
                                     size_t max_num_configuration_items)
{
    WOLK_UNUSED(max_num_configuration_items);
    WOLK_ASSERT(max_num_configuration_items >= NUMBER_OF_CONFIGURATION);

    strcpy(reference[0], "led_mode");
    strcpy(value[0], (led_mode==true ? "true" : "false"));
    UART_PRINT("Configuration Provider - Reference: %s | Value: %s\n\r", reference[0], value[0]);

    return CONFIGURATION_ITEMS_SIZE;
}


//****************************************************************************
//                            FIRMWARE HANDLERS
//****************************************************************************
bool firmware_update_start(const char* file_name, size_t file_size)
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

bool firmware_chunk_write(uint8_t* data, size_t data_size)
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


size_t firmware_chunk_read(size_t index, uint8_t* data, size_t data_size)
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

void firmware_update_abort(void)
{
    UART_PRINT("Aborting firmware update! \n\r");
    firmware_file_offset = 0;
    packet_no = 0;
    if (firmware_file != NULL)
    {
        sl_FsClose(firmware_file, 0, 0, 0);
    }
}

void firmware_update_finalize(void)
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

//    RebootMCU();//TODO
}

bool firmware_update_persist_firmware_version(const char* version)
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

bool firmware_update_unpersist_firmware_version(char* version, size_t version_size)
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
