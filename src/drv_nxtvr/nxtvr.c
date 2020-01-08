/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2019 Vis3r.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/*  Nxtvr Driver */

// Using as code base: https://github.com/der-b/OpenHMD/tree/master/src/drv_sparkfun_9dof

#include "nxtvr.h"

static void update_device(ohmd_device *device)
{
    nxt_priv *priv = (nxt_priv *)device;

    int size = 0;
    unsigned char buffer[FEATURE_BUFFER_SIZE];

    if (!priv->handle)
    {
        LOGE("NXTVR: Device handle missing in update_device fnc!");
        return;
    }

    while (true)
    {
        int size = hid_read(priv->handle, buffer, FEATURE_BUFFER_SIZE);
        if (size < 0)
        {
            LOGW("NXTVR: update_device - error reading device.");
            return;
        }
        else if (size == 0)
        {
            return;
        }
        handle_nxtvr_sensor_msg(priv, buffer, size);
    }
}

static int getf(ohmd_device *device, ohmd_float_value type, float *out)
{
    nxt_priv *priv = (nxt_priv *)device;

    switch (type)
    {
    case OHMD_ROTATION_QUAT:
        *(quatf *)out = priv->sensor_fusion.orient;
        break;
    case OHMD_POSITION_VECTOR:
        out[0] = out[1] = out[2] = 0;
        break;
    case OHMD_DISTORTION_K:
        memset(out, 0, sizeof(float) * 6);
        break;
    case OHMD_CONTROLS_STATE:
        out[0] = .1f;
        out[1] = 1.0f;
        break;
    default:
        ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
        return OHMD_S_INVALID_PARAMETER;
        break;
    }
    return OHMD_S_OK;
}

static void close_device(ohmd_device *device)
{
    nxt_priv *priv = (nxt_priv *)device;
    LOGD("closing nxtvr device");
    if (!priv->handle)
    {
        hid_close(priv->handle);
        priv->handle = NULL;
    }

    free(device);
}

static ohmd_device *open_device(ohmd_driver *driver, ohmd_device_desc *desc)
{
    nxt_priv *priv = ohmd_alloc(driver->ctx, sizeof(nxt_priv));

    if (!priv)
        return NULL;

    priv->handle = hid_open_path(desc->path);
    if (!priv->handle)
    {
        LOGD("Could not open directory "DEVICE_NAME);
        goto err;
    }

    if (hid_set_nonblocking(priv->handle, 1) == -1)
    {
        LOGD("Could not set nonblocking on "DEVICE_NAME);
        goto err_handle;
    }

    // Set default device properties
    ohmd_set_default_device_properties(&priv->base.properties);

    // Set device properties (imitates the rift values)
    priv->base.properties.hsize = 0.128490f;
    priv->base.properties.vsize = 0.070940f;
    priv->base.properties.hres = 1440;
    priv->base.properties.vres = 2560;
    priv->base.properties.lens_sep = 0.063500f;
    priv->base.properties.lens_vpos = 0.070940f / 2;
    priv->base.properties.fov = DEG_TO_RAD(103.0f);
    priv->base.properties.ratio = (2560.0f / 1440.0f) / 2.0f;

    // calculate projection eye projection matrices from the device properties
    ohmd_calc_default_proj_matrices(&priv->base.properties);

    // set up device callbacks
    priv->base.update = update_device;
    priv->base.close = close_device;
    priv->base.getf = getf;

    ofusion_init(&priv->sensor_fusion);
    priv->sensor_sync = 0;

    return (ohmd_device *)priv;
err_handle:
    hid_close(priv->handle);
    priv->handle = NULL;
err:
    free(priv);
    return NULL;
}

static void get_device_list(ohmd_driver *driver, ohmd_device_list *list)
{
    int id = 0;
    struct hid_device_info *devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
    struct hid_device_info *cur_dev = devs;
    ohmd_device_desc *desc;

    // HMD
    while (cur_dev)
    {
        printf("id: %d\n", id);
        desc = &list->devices[list->num_devices++];

        strcpy(desc->driver, "NxtVR Driver");
        strcpy(desc->vendor, "Vis3r");
        strcpy(desc->product, "NxtVR HMD");

        strncpy(desc->path, cur_dev->path, OHMD_STR_SIZE);

        desc->driver_ptr = driver;

        desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;
        desc->device_class = OHMD_DEVICE_CLASS_HMD;

        desc->id = id++;
        cur_dev = cur_dev->next;
    }

    hid_free_enumeration(devs);
}

static void destroy_driver(ohmd_driver *drv)
{
    //LOGD("Shutting down driver "DEVICE_NAME);
    free(drv);
}

ohmd_driver *ohmd_create_nxtvr_drv(ohmd_context *ctx)
{
    //LOGD("NXTVR: create HMD driver");
    ohmd_driver *drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
    if (!drv)
        return NULL;

    drv->get_device_list = get_device_list;
    drv->open_device = open_device;
    drv->destroy = destroy_driver;

    return drv;
}
