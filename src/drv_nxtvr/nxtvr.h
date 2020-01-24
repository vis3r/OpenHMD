#ifndef NXTVR_H
#define NXTVR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <hidapi.h>

#include "../openhmdi.h"
#include "../omath.h"

#define FEATURE_BUFFER_SIZE 256

//#define HID_REPORT_CONFRES_ID 1
//#define HID_REPORT_CONFRES_SIZE 32

#define HID_REPORT_MOTION_ID 1
#define HID_REPORT_MOTION_SIZE 32

// These ID's should be used only for debugging purposes
#define DEFAULT_STM32_VENDOR_ID 0x1eaf  // Default stm32duino VID
#define DEFAULT_STM32_PRODUCT_ID 0x0024 // Default stm32duino PID

#define VENDOR_ID 0x1209
#define PRODUCT_ID 0x9D0F

#define DEVICE_NAME "NxtVR HMD"


typedef struct
{
    ohmd_device base;
    hid_device *handle;

    fusion sensor_fusion;
    vec3f raw_accel, raw_gyro, raw_mag;
    int sensor_sync;
    uint64_t tick;
    float Scale[3];

} nxt_priv;

void handle_nxtvr_sensor_msg(nxt_priv *priv, const unsigned char *buffer, int size);

#endif
