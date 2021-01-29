#pragma once

#include "hal_usb_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

int hal_usb_init(void);
int hal_usb_attach(void);
int hal_usb_deattach(void);

#ifdef __cplusplus
}
#endif

