/*
 * Duplex matrix kscan driver for ZMK.
 *
 * Phase 1 (row2col): drives each row LOW, senses all cols with PULL_UP.
 *   Logical cols 0..(num_cols-1).
 * Phase 2 (col2row): drives each col LOW, senses all rows with PULL_UP.
 *   Logical cols num_cols..(2*num_cols-1).
 *
 * Both phases run sequentially in a single work queue poll, so the GPIO
 * pins never conflict.
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_duplex_matrix

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DUPLEX_MAX_ROWS 8
#define DUPLEX_MAX_COLS 8
/* settle time in microseconds after driving a pin LOW */
#define SETTLE_US 10

struct kscan_duplex_config {
    struct gpio_dt_spec rows[DUPLEX_MAX_ROWS];
    struct gpio_dt_spec cols[DUPLEX_MAX_COLS];
    uint8_t num_rows;
    uint8_t num_cols;
    uint32_t poll_period_ms;
};

struct kscan_duplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable poll_work;
    /* Bitmask of pressed cols per row; phase2 occupies upper bits */
    uint32_t matrix_state[DUPLEX_MAX_ROWS];
};

static void kscan_duplex_poll_handler(struct k_work *work)
{
    struct k_work_delayable *dwork =
        CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_duplex_data *data =
        CONTAINER_OF(dwork, struct kscan_duplex_data, poll_work);

    const struct device *dev = data->dev;
    const struct kscan_duplex_config *cfg = dev->config;

    /* --- Phase 1: rows drive LOW, cols sense --- */
    for (int c = 0; c < cfg->num_cols; c++) {
        gpio_pin_configure(cfg->cols[c].port, cfg->cols[c].pin,
                           GPIO_INPUT | GPIO_PULL_UP);
    }

    for (int r = 0; r < cfg->num_rows; r++) {
        gpio_pin_configure(cfg->rows[r].port, cfg->rows[r].pin,
                           GPIO_OUTPUT_LOW);
        k_busy_wait(SETTLE_US);

        for (int c = 0; c < cfg->num_cols; c++) {
            int val = gpio_pin_get(cfg->cols[c].port, cfg->cols[c].pin);
            bool pressed = (val == 0);
            bool was = (data->matrix_state[r] >> c) & 1U;
            if (pressed != was) {
                if (pressed) {
                    data->matrix_state[r] |= BIT(c);
                } else {
                    data->matrix_state[r] &= ~BIT(c);
                }
                LOG_DBG("P1 row=%d col=%d %s", r, c, pressed ? "PRESS" : "RELEASE");
                if (data->callback) {
                    data->callback(dev, r, c, pressed);
                }
            }
        }

        /* float row so phase 2 can use it as sense */
        gpio_pin_configure(cfg->rows[r].port, cfg->rows[r].pin, GPIO_INPUT);
    }

    /* --- Phase 2: cols drive LOW, rows sense --- */
    for (int r = 0; r < cfg->num_rows; r++) {
        gpio_pin_configure(cfg->rows[r].port, cfg->rows[r].pin,
                           GPIO_INPUT | GPIO_PULL_UP);
    }

    for (int c = 0; c < cfg->num_cols; c++) {
        gpio_pin_configure(cfg->cols[c].port, cfg->cols[c].pin,
                           GPIO_OUTPUT_LOW);
        k_busy_wait(SETTLE_US);

        int vcol = cfg->num_cols + c;

        for (int r = 0; r < cfg->num_rows; r++) {
            int val = gpio_pin_get(cfg->rows[r].port, cfg->rows[r].pin);
            bool pressed = (val == 0);
            bool was = (data->matrix_state[r] >> vcol) & 1U;
            if (pressed != was) {
                if (pressed) {
                    data->matrix_state[r] |= BIT(vcol);
                } else {
                    data->matrix_state[r] &= ~BIT(vcol);
                }
                LOG_DBG("P2 row=%d col=%d(vc%d) %s", r, c, vcol, pressed ? "PRESS" : "RELEASE");
                if (data->callback) {
                    data->callback(dev, r, vcol, pressed);
                }
            }
        }

        /* float col */
        gpio_pin_configure(cfg->cols[c].port, cfg->cols[c].pin, GPIO_INPUT);
    }

    k_work_reschedule(&data->poll_work, K_MSEC(cfg->poll_period_ms));
}

static int kscan_duplex_configure(const struct device *dev,
                                   kscan_callback_t callback)
{
    struct kscan_duplex_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int kscan_duplex_enable_callback(const struct device *dev)
{
    struct kscan_duplex_data *data = dev->data;
    k_work_reschedule(&data->poll_work, K_NO_WAIT);
    return 0;
}

static int kscan_duplex_disable_callback(const struct device *dev)
{
    struct kscan_duplex_data *data = dev->data;
    k_work_cancel_delayable(&data->poll_work);
    return 0;
}

static int kscan_duplex_init(const struct device *dev)
{
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *cfg = dev->config;

    data->dev = dev;
    memset(data->matrix_state, 0, sizeof(data->matrix_state));

    for (int r = 0; r < cfg->num_rows; r++) {
        if (!gpio_is_ready_dt(&cfg->rows[r])) {
            LOG_ERR("Row GPIO %d not ready", r);
            return -ENODEV;
        }
        gpio_pin_configure(cfg->rows[r].port, cfg->rows[r].pin, GPIO_INPUT);
    }
    for (int c = 0; c < cfg->num_cols; c++) {
        if (!gpio_is_ready_dt(&cfg->cols[c])) {
            LOG_ERR("Col GPIO %d not ready", c);
            return -ENODEV;
        }
        gpio_pin_configure(cfg->cols[c].port, cfg->cols[c].pin, GPIO_INPUT);
    }

    k_work_init_delayable(&data->poll_work, kscan_duplex_poll_handler);

    return 0;
}

static const struct kscan_driver_api kscan_duplex_api = {
    .config = kscan_duplex_configure,
    .enable_callback = kscan_duplex_enable_callback,
    .disable_callback = kscan_duplex_disable_callback,
};

#define GPIO_SPEC_ELEM(node_id, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)

#define KSCAN_DUPLEX_DEFINE(n)                                                  \
    static const struct kscan_duplex_config kscan_duplex_cfg_##n = {            \
        .rows = {                                                                \
            DT_INST_FOREACH_PROP_ELEM_SEP(n, row_gpios, GPIO_SPEC_ELEM, (,))   \
        },                                                                       \
        .cols = {                                                                \
            DT_INST_FOREACH_PROP_ELEM_SEP(n, col_gpios, GPIO_SPEC_ELEM, (,))   \
        },                                                                       \
        .num_rows = DT_INST_PROP_LEN(n, row_gpios),                             \
        .num_cols = DT_INST_PROP_LEN(n, col_gpios),                             \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                      \
    };                                                                           \
    static struct kscan_duplex_data kscan_duplex_data_##n;                      \
    DEVICE_DT_INST_DEFINE(n, kscan_duplex_init, NULL,                           \
                          &kscan_duplex_data_##n, &kscan_duplex_cfg_##n,         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,       \
                          &kscan_duplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_DUPLEX_DEFINE)
