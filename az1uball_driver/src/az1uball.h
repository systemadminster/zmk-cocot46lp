#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

/* Bit Masks */
#define MSK_SWITCH_STATE    0b10000000

struct az1uball_config {
    struct i2c_dt_spec i2c;
    /* Axis orientation / scaling (adopted from te9no/zmk-driver-az1uball).
     * Applied after smoothing: swap-xy, then per-axis invert, then scale. */
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    int scale_x;
    int scale_y;
};

struct az1uball_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool sw_pressed;
    bool sw_pressed_prev;
    uint32_t idle_count;

    /* Acceleration residual accumulator (Q8 fixed-point, 256 = 1.0). Carries
     * the sub-pixel remainder of the accelerated delta so slow/fine motion is
     * never dropped and alternating jitter averages out. See ACCEL_* in
     * az1uball.c. */
    int32_t resid_x;
    int32_t resid_y;
};
