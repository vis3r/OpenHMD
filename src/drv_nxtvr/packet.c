// Copyright 2017, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Original implementation by: Yann Vernier.
 */

/* Nxtvr - Packet Decoding and Utilities */

#include "nxtvr.h"

#ifdef _MSC_VER
#define inline __inline
#endif

inline static uint8_t read8(const unsigned char **buffer)
{
    uint8_t ret = **buffer;
    *buffer += 1;
    return ret;
}

inline static int16_t read16(const unsigned char **buffer)
{
    int16_t ret = **buffer | (*(*buffer + 1) << 8);
    *buffer += 2;
    return ret;
}

inline static uint32_t read32(const unsigned char **buffer)
{
    uint32_t ret = **buffer | (*(*buffer + 1) << 8) | (*(*buffer + 2) << 16) | (*
                   (*buffer + 3) << 24);
    *buffer += 4;
    return ret;
}

void nxtvr_handle_confRes_report(nxt_priv *priv, const unsigned char *buff)
{
    int32_t sample;
    for (int i = 0; i < 3; i++)
    {
        priv->Scale[i] = 1.0f;
    }
}

void accel_from_nxtvr_vec(const int32_t *smp, vec3f *out_vec, nxt_priv *priv)
{
    float out[3];
    for (int i = 0; i < 3; i++)
    {
        out[i] = (float)(smp[i] - 1073741824) / 2147483648;
    }
    out_vec->x = (float)out[0] * (1.0 / priv->Scale[0]);
    out_vec->y = (float)out[1] * (1.0 / priv->Scale[0]);
    out_vec->z = (float)out[2] * (1.0 / priv->Scale[0]);
}

void nxtvr_handle_accel_report(nxt_priv *priv, const unsigned char *buff)
{
    int32_t sample[3];
    for (int i = 0; i < 3; i++)
    {
        sample[i] = (int32_t)read32(&buff);
    };

    accel_from_nxtvr_vec(sample, &priv->raw_accel, priv);
}

void gyro_from_nxtvr_vec(const int32_t *smp, vec3f *out_vec, nxt_priv *priv)
{
    //Based on the scale factor of the MPU6050, by default, this would be 131,
    //which is the case in our config
    //TODO: Look for the scale factor of the BMX
    float out_f[3];
    for (int i = 0; i < 3; i++)
    {
        out_f[i] = (float)(smp[i] - 1073741824) / 2147483648;
    }
    out_vec->x = (float)out_f[0] * (1.0 / priv->Scale[1]);
    out_vec->y = (float)out_f[1] * (1.0 / priv->Scale[1]);
    out_vec->z = (float)out_f[2] * (1.0 / priv->Scale[1]);
}

void nxtvr_handle_gyro_report(nxt_priv *priv, const unsigned char *buff)
{
    int32_t sample[3];

    for (int i = 0; i < 3; i++)
    {
        sample[i] = (int32_t)read32(&buff);
    }

    gyro_from_nxtvr_vec(sample, &priv->raw_gyro, priv);
}

void handle_nxtvr_sensor_msg(nxt_priv *priv, const unsigned char *buffer,
                             int size)
{
    if (size < 7)
    {
        LOGE("NXTVR: REPORT size has to be at least 7!");
        return;
    }

    uint8_t reportID = read8(&buffer);
    LOGD("Report ID: %d", reportID);

    switch (reportID)
    {
    /*    case HID_REPORT_CONFRES_ID:
            nxtvr_handle_confRes_report(priv, buffer);
            priv->sensor_sync++;
            break;
    */
    case HID_REPORT_MOTION_ID:
        nxtvr_handle_accel_report(priv, buffer);
        nxtvr_handle_gyro_report(priv, buffer);
        priv->sensor_sync++;
        break;

    default:
        LOGE("NXTVR: Unknown reportID %d ", reportID);
        return;
    }


    // all sensor info is sent in it's own
    // descriptor, be sure we have received both
    // before doing fusion
    // TODO: put accel and gyro together
    if (priv->sensor_sync >= 1)
    {
        priv->sensor_sync = 0;
        uint64_t last_sample_tick = priv->tick;
        priv->tick = ohmd_monotonic_get(priv->base.ctx);

        // Startup correction, ignore last_sample_tick if zero.
        uint64_t tick_delta = 0;
        if (last_sample_tick > 0) //startup correction
            tick_delta = priv->tick - last_sample_tick;

        float dt = tick_delta / 1000000000000.0f;

        ofusion_update(&priv->sensor_fusion, dt, &priv->raw_gyro, &priv->raw_accel,
                       &priv->raw_mag);
    }
}
