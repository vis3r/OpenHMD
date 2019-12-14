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
	uint32_t ret = **buffer | (*(*buffer + 1) << 8) | (*(*buffer + 2) << 16) | (*(*buffer + 3) << 24);
	*buffer += 4;
	return ret;
}

void accel_from_nxtvr_vec(const int16_t *smp, vec3f *out_vec)
{
	out_vec->x = (float)smp[0] * (2.0 / 2048.0);
	out_vec->y = (float)smp[1] * (2.0 / 2048.0);
	out_vec->z = (float)smp[2] * (2.0 / 2048.0);
}

void nxtvr_handle_accel_report(nxt_priv *priv, const unsigned char *buff)
{
	int16_t sample[3];
	for (int i = 0; i < 3; i++)
	{
		sample[i] = read16(&buff);
	};

	accel_from_nxtvr_vec(sample, &priv->raw_accel);
}

void gyro_from_nxtvr_vec(const int16_t *smp, vec3f *out_vec)
{
	//Based on the scale factor of the MPU6050, by default, this would be 131,
	//which is the case in our config
	//TODO: Look for the scale factor of the BMX
	out_vec->x = (float)smp[0] * (1.0 / 131.0);
	out_vec->y = (float)smp[1] * (1.0 / 131.0);
	out_vec->z = (float)smp[2] * (1.0 / 131.0);
}

void nxtvr_handle_gyro_report(nxt_priv *priv, const unsigned char *buff)
{
	int16_t sample[3];

	for (int i = 0; i < 3; i++)
	{
		sample[i] = read16(&buff);
	}

	gyro_from_nxtvr_vec(sample, &priv->raw_gyro);
};

void handle_nxtvr_sensor_msg(nxt_priv *priv, const unsigned char *buffer, int size)
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
	case HID_REPORT_ACCEL_ID:
		nxtvr_handle_accel_report(priv, buffer);
		priv->sensor_sync++;
		break;

	case HID_REPORT_GYRO_ID:
		nxtvr_handle_gyro_report(priv, buffer);
		priv->sensor_sync++;
		break;

	case HID_REPORT_MAG_ID:
		return;

	default:
		LOGE("NXTVR: Unknown reportID %d ", reportID);
		return;
	}


	// all sensor info is sent in it's own
	// descriptor, be sure we have received both
	// before doing fusion
	// TODO: put accel and gyro together
	if (priv->sensor_sync >= 2)
	{
		priv->sensor_sync = 0;
		uint64_t last_sample_tick = priv->tick;
		priv->tick = ohmd_monotonic_get(priv->base.ctx);

		// Startup correction, ignore last_sample_tick if zero.
		uint64_t tick_delta = 0;
		if (last_sample_tick > 0) //startup correction
			tick_delta = priv->tick - last_sample_tick;

		float dt = tick_delta / 1000000000000.0f;

		ofusion_update(&priv->sensor_fusion, dt, &priv->raw_gyro, &priv->raw_accel, &priv->raw_mag);
	}
}
