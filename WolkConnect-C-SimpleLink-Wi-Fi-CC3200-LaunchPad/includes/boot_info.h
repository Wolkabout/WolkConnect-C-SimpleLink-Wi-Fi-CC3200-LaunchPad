#ifndef BOOT_INFO_H_
#define BOOT_INFO_H_

#include <stdint.h>
#include "simplelink.h"
#include "hw_types.h"
#include "flc.h"
#include "fs.h"
#include "uart_if.h"

/**
 * \brief Function which writes boot info to /sys/mcuinfo.bin
 *
 * \param psBootInfo struct which contains active image and image status flags
 * \return 0 success, other error
 */
_i32 WriteBootInfo(sBootInfo_t *psBootInfo);

/**
 * \brief Function which reads boot info to /sys/mcuinfo.bin
 *
 * \param psBootInfo struct which contains active image and image status flags
 * \return 0 success, other error
 */
_i32 ReadBootInfo(sBootInfo_t *psBootInfo);

#endif /* BOOT_INFO_H_ */
