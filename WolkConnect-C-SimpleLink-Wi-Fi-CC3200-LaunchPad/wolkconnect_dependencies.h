/*
 * wolkconnect_dependencies.h
 *
 *  Created on: 22 May 2019
 *      Author: srdjan.stankovic
 */

#ifndef WOLKCONNECT_DEPENDENCIES_H_
#define WOLKCONNECT_DEPENDENCIES_H_

#define USER_IMAGE_1            "/sys/mcuimg2.bin"
#define USER_IMAGE_2            "/sys/mcuimg3.bin"
#define FIRMWARE_CHUNK_SIZE     512

static int total_chunk_count = 0;
static int packet_no = 0;

static long firmware_file;
static size_t firmware_file_size = 0;
static unsigned long firmware_file_offset = 0;
static const char* firmware_version_file = "firmware_version";
static const int16_t firmware_version_file_size = 20;

const int16_t firmware_chunk_size;// = FIRMWARE_CHUNK_SIZE;

void actuation_handler(const char* reference, const char* value);
actuator_status_t actuator_status_provider(const char* reference);

void configuration_handler(char (*reference)[CONFIGURATION_REFERENCE_SIZE],
                                  char (*value)[CONFIGURATION_VALUE_SIZE],
                                  size_t num_configuration_items);
size_t configuration_provider(char (*reference)[CONFIGURATION_REFERENCE_SIZE],
                                     char (*value)[CONFIGURATION_VALUE_SIZE],
                                     size_t max_num_configuration_items);

bool firmware_update_start(const char* file_name, size_t file_size);
bool firmware_chunk_write(uint8_t* data, size_t data_size);
size_t firmware_chunk_read(size_t index, uint8_t* data, size_t data_size);
void firmware_update_abort(void);
void firmware_update_finalize(void);
bool firmware_update_persist_firmware_version(const char* version);
bool firmware_update_unpersist_firmware_version(char* version, size_t version_size);


#endif /* WOLKCONNECT_DEPENDENCIES_H_ */
