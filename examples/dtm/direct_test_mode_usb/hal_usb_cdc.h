#pragma once

#include <stdint.h>
#include <stddef.h>
#include "nrf_assert.h"

#define SYSTEM_ERROR_NONE 0
#define SYSTEM_ERROR_INVALID_STATE -1
#define SYSTEM_ERROR_NO_MEMORY -2

#define SPARK_ASSERT(result)    \
do { \
    if (!result) { \
        while(1); \
    }\
} while(0)


#ifdef __cplusplus
extern "C" {
#endif

int hal_usb_cdc_init();
int hal_usb_cdc_available_for_read();
int hal_usb_cdc_available_for_write();
int hal_usb_cdc_read(uint8_t* buf, size_t size);
int hal_usb_cdc_write(uint8_t* buf, size_t size);

#ifdef __cplusplus
}
#endif
