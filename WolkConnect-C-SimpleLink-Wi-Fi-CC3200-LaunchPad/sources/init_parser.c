#include "init_parser.h"

static const char *valid_options[] = {"ssid_name", "ssid_password",
                                      "device_key", "device_password"};                     //!< Options which are checked when parsing file
static const char delimiter = '=';                                                          //!< Delimiter option=value

static parser_state_t parser_state;                                                         //!< Helper var which reperesent parser state


/**
 * \brief Helper function checks value and adds it to parser state
 *
 * \param value String read from file
 * \param option Index of option from valid_options
 */
static void add_value(char* value, int8_t option);

/**
 * \brief Helper function checks if option is valid
 *
 * \param option Option string read from file
 *
 * \return -1 option is not valid, else index of option in valid_options
 */
static int8_t check_option(char* option);


/**
 * \brief Helper function checks if all values are read
 *
 * \return 0 all values are read, else -1
 */
static int8_t all_values_read();

/**
 * \brief Helper function converts ip_addr string to int64_t
 *
 * \param ip_addr_str string to be converted
 *
 * \param num_addr result to be stored
 *
 * \return 0 conversion succesful, else -1
 */
static int8_t ip_to_num(const char* ip_addr_str, int64_t* num_addr);

/**
 * \brief Helper function converts string to int16_t
 *
 * \param str string to be converted
 *
 * \param num result to be stored
 *
 * \return 0 conversion succesful, else -1
 */
int8_t my_atoi(const char *str, int16_t* num);

int8_t init_file_parse(const char* file_path)
{
    long file_handle;
    int lRetVal = 0;
    int64_t read_pos = 0;

    lRetVal = sl_FsOpen((unsigned char*) file_path,
                            FS_MODE_OPEN_READ,
                             NULL,
                             &file_handle);
    if (lRetVal < 0)
    {
        UART_PRINT("Couldn't open %s for reading \n\r", file_path);
        sl_FsClose(file_handle, 0, 0, 0);
        return -1;
    }

    // Parse file line by line
    unsigned char current_char;
    char option[MAX_VALUE_LENGTH] = {'\0'};
    char value[MAX_VALUE_LENGTH] = {'\0'};
    int8_t i = 0;
    int8_t reading_option = 1;
    do
    {
        int8_t read_bytes;
        read_bytes = sl_FsRead(file_handle,
                                  (unsigned int)read_pos,
                                  &current_char,
                                  sizeof(char));
        // EOF
        if (read_bytes < (int8_t)sizeof(char))
        {
            value[i] = '\0';
            i = 0;
            int8_t current_option = check_option(option);
            if (current_option == -1)
            {
                UART_PRINT("unknown option %s \n", option);
                fflush(stdout);
                return 1;
            }
            add_value(value, current_option);
            break;
        }
        read_pos++;
        // Check if char is delimiter
        if (current_char == delimiter) {
            reading_option = 0;
            option[i] = '\0';
            i = 0;
        // Check if char is end of line
        } else if (current_char == '\n' || current_char == 13 /* windows carriage return */)
        {
            reading_option = 1;
            value[i] = '\0';
            i = 0;
            int8_t current_option = check_option(option);
            if (current_option == -1)
            {
                UART_PRINT("unknown option %s \n", option);
                fflush(stdout);
                return 1;
            }
            add_value(value, current_option);
            if (current_char == 13) {
                // On windows, new line is following after carriage return, skip it
                read_pos++;
            }
        } else
        {
            if(reading_option)
            {
                option[i] = current_char;
            } else
            {
                value[i] = current_char;
            }
            i++;
        }
    } while (1);

    sl_FsClose(file_handle, 0, 0, 0);

    return all_values_read();
}

char* parser_get_ssid_name()
{
    if (parser_state.ssid_name.is_read == 1)
    {
        return parser_state.ssid_name.value;
    } else
    {
        return NULL;
    }
}

char* parser_get_ssid_password()
{
    if (parser_state.ssid_password.is_read == 1)
    {
        return parser_state.ssid_password.value;
    } else
    {
        return NULL;
    }
}

char* parser_get_device_key()
{
    if (parser_state.device_key.is_read == 1)
    {
        return parser_state.device_key.value;
    } else
    {
        return NULL;
    }
}

char* parser_get_device_password()
{
    if (parser_state.device_password.is_read == 1)
    {
        return parser_state.device_password.value;
    } else
    {
        return NULL;
    }
}

static int8_t ip_to_num(const char* ip_addr_str, int64_t* num_addr)
{
    int8_t read_values = 0;
    uint8_t i;
    int16_t current_num;
    int8_t shift = 3;
    *num_addr = 0;
    char read_num[4] = {};
    int k = 0;
    int bz = strlen(ip_addr_str);

    for (i = 0; i < strlen(ip_addr_str); i++)
    {
        // Check valid character
        if (ip_addr_str[i] >= '0' && ip_addr_str[i] <= '9')
        {
            read_num[k++] = ip_addr_str[i];
        } else if (ip_addr_str[i] == '.')
        {
            read_num[k] = '\0';
            my_atoi(read_num, &current_num);
            *num_addr += ((int64_t)current_num) << (8*shift);
            shift--;
            read_values++;
            k = 0;
        } else
        {
            return -1;
        }
    }

    // Last number from ip
    read_num[k] = '\0';
    my_atoi(read_num, &current_num);
    *num_addr += ((int64_t)current_num) << (8*shift);
    shift--;
    read_values++;

    if(read_values != 4) {
        return -1;
    }

    return 0;
}

int8_t my_atoi(const char *str, int16_t* num)
{
    int i = 0;
    *num = 0;

    // Iterate through all characters of input string and
    // update result
    for (i = 0; i < strlen(str); i++)
    {
        if (str[i] >= '0' && str[i] <= '9')
        {
            *num = *num*10 + str[i] - '0';
        } else
        {
            int k = strlen(str);
            return -1;
        }
    }

    return 0;
}

static int8_t all_values_read()
{
    return !(parser_state.ssid_name.is_read && parser_state.ssid_password.is_read
            && parser_state.device_key.is_read && parser_state.device_password.is_read);
}

static int8_t check_option(char* option)
{
    uint8_t i;
    for (i = 0; i < NUMBER_OF_OPTIONS; i++)
    {
        if (strcmp(option, valid_options[i]) == 0)
        {
            return i;
        }
    }
    return -1;
}

static void add_value(char* value, int8_t option)
{
    switch (option)
    {
    case 0:
        if (strlen(value) > 0)
        {
            parser_state.ssid_name.is_read = 1;
            strcpy(parser_state.ssid_name.value, value);
        } else
        {
            parser_state.ssid_name.is_read = 0;
        }
        break;
    case 1:
        if (strlen(value) > 0)
        {
            parser_state.ssid_password.is_read = 1;
            strcpy(parser_state.ssid_password.value, value);
        } else
        {
            parser_state.ssid_password.is_read = 0;
        }
        break;
    case 2:
        if (strlen(value) > 0)
        {
            parser_state.device_key.is_read = 1;
            strcpy(parser_state.device_key.value, value);
        } else
        {
            parser_state.device_key.is_read = 0;
        }
        break;
    case 3:
        if (strlen(value) > 0)
        {
            parser_state.device_password.is_read = 1;
            strcpy(parser_state.device_password.value, value);
        } else
        {
            parser_state.device_password.is_read = 0;
        }
        break;
    }
}


