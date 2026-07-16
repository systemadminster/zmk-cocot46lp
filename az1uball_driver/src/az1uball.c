/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT palette_az1uball

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include "az1uball.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(az1uball, CONFIG_AZ1UBALL_LOG_LEVEL);

#define POLL_INTERVAL       K_MSEC(10)   // アクティブ時: 10ms (100Hz)
#define POLL_INTERVAL_IDLE  K_MSEC(100)  // アイドル時: 100ms (10Hz)
#define IDLE_THRESHOLD      50           // 500ms 無操作でアイドルへ移行

/*
 * Pointer acceleration + residual accumulation (replaces the old low-pass).
 *
 * The kzyz driver has no CPI/accel knob and just reported the raw count, which
 * the x6 scaler then quantised into coarse 6px steps -- fine control snagged and
 * the earlier low-pass made it worse by rounding small moves away entirely.
 *
 * Instead, scale each poll's delta by a speed-dependent multiplier (Q8 fixed
 * point, 256 = 1.0x):
 *
 *   speed = |dx| + |dy|                    (movement this 10ms poll)
 *   mult  = BASE + GAIN * speed  (capped at MAX)
 *   out   = delta * mult                   (accumulated with a residual)
 *
 * Slow motion -> small mult -> precise, sub-pixel-smooth fine control, and the
 * alternating +/-1 sensor jitter is barely amplified AND averages out in the
 * residual (so no laggy filter is needed). Fast flicks -> large mult -> quick.
 * The residual carries the fractional pixels so no motion is ever dropped.
 *
 * Tuning: raise BASE for faster slow-speed tracking (less fine); raise
 * GAIN/MAX for faster flicks; lower BASE for finer control + more jitter reject.
 */
#define ACCEL_BASE_Q8   179   /* ~0.70x at the slowest motion (fine control)   */
#define ACCEL_GAIN_Q8   205   /* +~0.80x per unit of per-poll speed            */
#define ACCEL_MAX_Q8   2304   /* cap at 9.0x for fast flicks (was a flat 6x)   */

/* Execution functions for asynchronous work */
static void az1uball_work_handler(struct k_work *work)
{
    struct az1uball_data *data = CONTAINER_OF(k_work_delayable_from_work(work), struct az1uball_data, work);
    const struct az1uball_config *config = data->dev->config;
    uint8_t buf[5];
    int ret;

    // Read data from I2C
    ret = i2c_read_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read movement data from AZ1UBALL: %d", ret);
        k_work_reschedule(&data->work, POLL_INTERVAL);
        return;
    }

    /* Calculate deltas */
    int16_t delta_x = (int16_t)buf[2] - (int16_t)buf[3]; // RIGHT - LEFT
    int16_t delta_y = (int16_t)buf[1] - (int16_t)buf[0]; // DOWN - UP

    /* Speed-dependent acceleration with residual accumulation (see note above). */
    int aspeed = (delta_x < 0 ? -delta_x : delta_x) + (delta_y < 0 ? -delta_y : delta_y);
    int32_t mult = ACCEL_BASE_Q8 + ACCEL_GAIN_Q8 * aspeed;
    if (mult > ACCEL_MAX_Q8) {
        mult = ACCEL_MAX_Q8;
    }

    data->resid_x += (int32_t)delta_x * mult;
    data->resid_y += (int32_t)delta_y * mult;

    int out_x = data->resid_x >> 8;          /* whole pixels out this poll */
    int out_y = data->resid_y >> 8;
    data->resid_x -= (int32_t)out_x << 8;     /* keep the sub-pixel remainder */
    data->resid_y -= (int32_t)out_y << 8;

    /* Axis orientation / scaling (te9no-style): correct a rotated/mirrored
     * mounting purely from devicetree. Order: swap-xy, invert, scale. */
    if (config->swap_xy) {
        int tmp = out_x;
        out_x = out_y;
        out_y = tmp;
    }
    if (config->invert_x) {
        out_x = -out_x;
    }
    if (config->invert_y) {
        out_y = -out_y;
    }
    out_x *= config->scale_x;
    out_y *= config->scale_y;

    /* Idle tracking follows the RAW sensor, so polling stays at 100Hz while the
     * ball is physically moving even if the accelerated output is 0 this poll. */
    if (delta_x != 0 || delta_y != 0) {
        data->idle_count = 0;
    } else if (data->idle_count < IDLE_THRESHOLD) {
        data->idle_count++;
    }

    /* Report accelerated movement if non-zero */
    if (out_x != 0 || out_y != 0) {
        /* Report relative X movement */
        if (out_x != 0) {
            bool sync = (out_y == 0);
            ret = input_report_rel(data->dev, INPUT_REL_X, out_x, sync, K_NO_WAIT);
            if (ret < 0) {
                LOG_ERR("Failed to report delta_x: %d", ret);
            } else {
                LOG_DBG("Reported delta_x: %d", out_x);
            }
        }

        /* Report relative Y movement */
        if (out_y != 0) {
            ret = input_report_rel(data->dev, INPUT_REL_Y, out_y, true, K_NO_WAIT);
            if (ret < 0) {
                LOG_ERR("Failed to report delta_y: %d", ret);
            } else {
                LOG_DBG("Reported delta_y: %d", out_y);
            }
        }
    }

    /* Update switch state */
    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    /* Report switch state if it changed */
    if (data->sw_pressed != data->sw_pressed_prev) {
        ret = input_report_key(data->dev, INPUT_BTN_0, data->sw_pressed ? 1 : 0, true, K_NO_WAIT);
        if (ret < 0) {
            LOG_ERR("Failed to report key");
        }

        LOG_DBG("Reported switch state: %d", data->sw_pressed);

        data->sw_pressed_prev = data->sw_pressed;
    }

    k_timeout_t interval = (data->idle_count >= IDLE_THRESHOLD) ? POLL_INTERVAL_IDLE : POLL_INTERVAL;
    k_work_reschedule(&data->work, interval);
}

static int az1uball_pm_action(const struct device *dev, enum pm_device_action action)
{
    struct az1uball_data *data = dev->data;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_work_cancel_delayable(&data->work);
        break;
    case PM_DEVICE_ACTION_RESUME:
        data->idle_count = 0;
        k_work_schedule(&data->work, POLL_INTERVAL);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

/* Initialization of AZ1UBALL */
static int az1uball_init(const struct device *dev)
{
    const struct az1uball_config *config = dev->config;
    struct az1uball_data *data = dev->data;
    int ret;

    data->dev = dev;
    data->sw_pressed_prev = false;
    data->resid_x = 0;
    data->resid_y = 0;

    /* Check if the I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready: 0x%x", config->i2c.addr);
        return -ENODEV;
    }

    /* Set turbo mode */
    uint8_t cmd = 0x91;
    ret = i2c_write_dt(&config->i2c, &cmd, sizeof(cmd));
    if (ret < 0) {
        LOG_ERR("Failed to set turbo mode");
        return ret;
    }

    k_work_init_delayable(&data->work, az1uball_work_handler);
    k_work_schedule(&data->work, POLL_INTERVAL);

    return 0;
}

#define AZ1UBALL_DEFINE(n)                                             \
    static struct az1uball_data az1uball_data_##n;                     \
    static const struct az1uball_config az1uball_config_##n = {        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                \
        .invert_x = DT_INST_PROP(n, invert_x),                         \
        .invert_y = DT_INST_PROP(n, invert_y),                         \
        .swap_xy = DT_INST_PROP(n, swap_xy),                           \
        .scale_x = DT_INST_PROP_OR(n, scale_x, 1),                     \
        .scale_y = DT_INST_PROP_OR(n, scale_y, 1),                     \
    };                                                                 \
    PM_DEVICE_DT_INST_DEFINE(n, az1uball_pm_action);                   \
    DEVICE_DT_INST_DEFINE(n,                                           \
                          az1uball_init,                               \
                          PM_DEVICE_DT_INST_GET(n),                    \
                          &az1uball_data_##n,                          \
                          &az1uball_config_##n,                        \
                          POST_KERNEL,                                 \
                          CONFIG_INPUT_INIT_PRIORITY,                  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(AZ1UBALL_DEFINE)
