#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pimoroni_pim447, CONFIG_SENSOR_LOG_LEVEL);

#define PIMORONI_PIM447_I2C_ADDRESS 0x0A

// Register addresses
#define REG_LEFT 0x00
#define REG_RIGHT 0x01
#define REG_UP 0x02
#define REG_DOWN 0x03
#define REG_SWITCH 0x04

struct pimoroni_pim447_data {
    const struct device *i2c_dev;
    int16_t dx;
    int16_t dy;
    bool pressed;
};

struct pimoroni_pim447_config {
    struct i2c_dt_spec i2c;
};

static int pimoroni_pim447_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t buf[5];
    int ret;

    ret = i2c_burst_read_dt(&cfg->i2c, REG_LEFT, buf, 5);
    if (ret < 0) {
        LOG_ERR("Failed to read movement data from PIM447");
        return ret;
    }

    // Calculate dx and dy
    int16_t dx = (int16_t)buf[1] - (int16_t)buf[0];
    int16_t dy = (int16_t)buf[3] - (int16_t)buf[2];

    // Update button state
    bool pressed = (buf[4] & 0x80) != 0;

    // Log movement if there's any change
    if (dx != 0 || dy != 0 || pressed != data->pressed) {
        LOG_INF("Trackball movement: dx=%d, dy=%d, pressed=%d", dx, dy, pressed);
    }

    // Update data
    data->dx = dx;
    data->dy = dy;
    data->pressed = pressed;

    return 0;
}


static int pimoroni_pim447_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct pimoroni_pim447_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->dx;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->dy;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_PROX:
        val->val1 = data->pressed ? 1 : 0;
        val->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api pimoroni_pim447_api = {
    .sample_fetch = pimoroni_pim447_sample_fetch,
    .channel_get = pimoroni_pim447_channel_get,
};

static int pimoroni_pim447_init(const struct device *dev)
{
    const struct pimoroni_pim447_config *cfg = dev->config;

    LOG_INF("pim447 am Start");

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    return 0;
}

#define PIMORONI_PIM447_INIT(inst)                                            \
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
                          &pimoroni_pim447_api);

DT_INST_FOREACH_STATUS_OKAY(PIMORONI_PIM447_INIT)