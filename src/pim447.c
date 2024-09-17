#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/pointer_movement.h>

LOG_MODULE_REGISTER(pimoroni_pim447, CONFIG_SENSOR_LOG_LEVEL);

#define PIMORONI_PIM447_I2C_ADDRESS 0x0A
#define CHIP_ID 0xBA11  // Replace with actual chip ID if known

struct pimoroni_pim447_data {
    const struct device *i2c_dev;
    const struct device *dev;
    int16_t delta_x;
    int16_t delta_y;
    bool sw_pressed;
    struct k_work work;
};

struct pimoroni_pim447_config {
    struct i2c_dt_spec i2c;
};

static int pimoroni_pim447_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t buf[5];
    int ret;

    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&cfg->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447");
        return ret;
    }

    data->delta_x = (int16_t)buf[1] - (int16_t)buf[0]; // Right - Left
    data->delta_y = (int16_t)buf[3] - (int16_t)buf[2]; // Down - Up

    uint8_t sw_state = buf[4];
    data->sw_pressed = (sw_state & MSK_SWITCH_STATE) != 0;

    return 0;
}

static void pimoroni_pim447_work_handler(struct k_work *work) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, work);

    if (pimoroni_pim447_sample_fetch(data->dev, SENSOR_CHAN_ALL) == 0) {
        /* Emit pointer movement event */
        struct zmk_pointer_movement_event *ev = new_zmk_pointer_movement_event();
        ev->dx = data->delta_x;
        ev->dy = data->delta_y;
        ev->pressed = data->sw_pressed;
        ZMK_EVENT_RAISE(ev);
    }
}

static int pimoroni_pim447_init(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t chip_id_l, chip_id_h;
    uint16_t chip_id;

    data->i2c_dev = cfg->i2c.bus;
    if (!device_is_ready(data->i2c_dev)) {
        LOG_ERR("I2C bus device is not ready");
        return -EINVAL;
    }

    data->dev = dev;

    /* Read Chip ID */
    if (i2c_reg_read_byte_dt(&cfg->i2c, REG_CHIP_ID_L, &chip_id_l) ||
        i2c_reg_read_byte_dt(&cfg->i2c, REG_CHIP_ID_H, &chip_id_h)) {
        LOG_ERR("Failed to read chip ID");
        return -EIO;
    }
    chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    if (chip_id != CHIP_ID) {
        LOG_ERR("Invalid chip ID: 0x%04X", chip_id);
        return -EINVAL;
    }

    LOG_INF("Pimoroni PIM447 detected, chip ID: 0x%04X", chip_id);

    k_work_init(&data->work, pimoroni_pim447_work_handler);

    /* Optionally, initialize the LED to a default state */
    // pimoroni_pim447_led_set(dev, 0, 0, 0, 0); // Turn off the LED initially

    /* Start periodic fetching */
    k_work_submit(&data->work);

    return 0;
}

static const struct sensor_driver_api pimoroni_pim447_driver_api = {
    .sample_fetch = pimoroni_pim447_sample_fetch,
    // .channel_get is not needed for ZMK pointer implementation
};

#define PIM447_INST(inst)                                                     \
    static struct pimoroni_pim447_data pimoroni_pim447_data_##inst;           \
    static const struct pimoroni_pim447_config pimoroni_pim447_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                    \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                               \
                          pimoroni_pim447_init,                               \
                          NULL,                                               \
                          &pimoroni_pim447_data_##inst,                       \
                          &pimoroni_pim447_config_##inst,                     \
                          POST_KERNEL,                                        \
                          CONFIG_SENSOR_INIT_PRIORITY,                        \
                          &pimoroni_pim447_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INST)
