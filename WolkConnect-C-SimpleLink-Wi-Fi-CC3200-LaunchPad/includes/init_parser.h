#ifndef INIT_PARSER_H
#define INIT_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "uart_if.h"

#include "fs.h"
#include "common.h"

#define MAX_VALUE_LENGTH    40      //!< Max number of characters read for value from config file
#define NUMBER_OF_OPTIONS   4       //!< Number of options

typedef struct init_options {
    char value[MAX_VALUE_LENGTH];   //!< Value which is read from init file
    uint8_t is_read;                //!< Flag, 0 if not read, else 1
} init_options_t;

typedef struct parser_s {
    init_options_t ssid_name;
    init_options_t ssid_password;
//    init_options_t platform_ip;
//    init_options_t platform_port;
    init_options_t device_key;
    init_options_t device_password;
} parser_state_t;


/*!
 * \brief Function which parses init file and stores values for options
 *
 * \param file_path Path to file which contains initial configuration
 *
 * \return 0 on success or 1 on error
 */
int8_t init_file_parse(const char* file_path);

/*!
 * \brief Get value for ssid_name which is read from init file
 *
 * \return NULL if ssid_name is not read, else string which is read
 */
char* parser_get_ssid_name();

/*!
 * \brief Get value for ssid_password which is read from init file
 *
 * \return NULL if ssid_password is not read, else string which is read
 */
char* parser_get_ssid_password();

/*!
 * \brief Get value for device_key which is read from init file
 *
 * \return NULL if device_key is not read, else string which is read
 */
char* parser_get_device_key();

/*!
 * \brief Get value for device_password which is read from init file
 *
 * \return NULL if device_password is not read, else string which is read
 */
char* parser_get_device_password();

#endif
