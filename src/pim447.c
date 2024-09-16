#define DT_DRV_COMPAT pimoroni_pim447

#include "pim447.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(pimoroni_pim447, CONFIG_SENSOR_LOG_LEVEL);

#define PIMORONI_PIM447_I2C_ADDRESS 0x0A
#define CHIP_ID 0xBA11  // Replace with actual chip ID if known

static int pimoroni_pim447_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t buf[5];
    int ret;

    /* Read movement data and switch state */
    ret = i2c_burst_read(data->i2c_dev, cfg->i2c_addr, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447");
        return ret;
    }

    data->delta_left = buf[0];
    data->delta_right = buf[1];
    data->delta_up = buf[2];
    data->delta_down = buf[3];

    uint8_t sw_state = buf[4];
    data->sw_changed = sw_state & ~MSK_SWITCH_STATE;
    data->sw_pressed = (sw_state & MSK_SWITCH_STATE) != 0;

    return 0;
}

static int pimoroni_pim447_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct pimoroni_pim447_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = (int16_t)data->delta_right - (int16_t)data->delta_left;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = (int16_t)data->delta_down - (int16_t)data->delta_up;
        break;
    case SENSOR_CHAN_PRESS:
        val->val1 = data->sw_pressed ? 1 : 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static void pimoroni_pim447_work_handler(struct k_work *work) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, work);

    if (pimoroni_pim447_sample_fetch(data->dev, SENSOR_CHAN_ALL) == 0) {
        // Process the data or generate an event
        // Integrate with ZMK's sensor framework to emit HID reports
    }
}

#ifdef CONFIG_ZMK_SENSOR_PIMORONI_PIM447_INTERRUPT
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);

    k_work_submit(&data->work);
}
#endif

static int pimoroni_pim447_init(const struct device *dev) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    uint8_t chip_id_l, chip_id_h;
    uint16_t chip_id;

    data->i2c_dev = cfg->i2c_bus;
    if (!device_is_ready(data->i2c_dev)) {
        LOG_ERR("I2C bus device is not ready");
        return -EINVAL;
    }

    data->dev = dev;  // Store the device pointer for use in work handler

    /* Read Chip ID */
    if (i2c_reg_read_byte(data->i2c_dev, cfg->i2c_addr, REG_CHIP_ID_L, &chip_id_l) ||
        i2c_reg_read_byte(data->i2c_dev, cfg->i2c_addr, REG_CHIP_ID_H, &chip_id_h)) {
        LOG_ERR("Failed to read chip ID");
        return -EIO;
    }
    chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    if (chip_id != CHIP_ID) {  // Replace CHIP_ID with the actual chip ID if known
        LOG_ERR("Invalid chip ID: 0x%04X", chip_id);
        return -EINVAL;
    }

    LOG_INF("Pimoroni PIM447 detected, chip ID: 0x%04X", chip_id);

#ifdef CONFIG_ZMK_SENSOR_PIMORONI_PIM447_INTERRUPT
    if (device_is_ready(cfg->int_gpio.port)) {
        int ret;

        ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure interrupt GPIO");
            return ret;
        }

        gpio_init_callback(&data->int_gpio_cb, pimoroni_pim447_gpio_callback, BIT(cfg->int_gpio.pin));
        ret = gpio_add_callback(cfg->int_gpio.port, &data->int_gpio_cb);
        if (ret) {
            LOG_ERR("Failed to add GPIO callback");
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret) {
            LOG_ERR("Failed to configure GPIO interrupt");
            return ret;
        }

        /* Enable interrupt output on the trackball */
        uint8_t int_reg;
        if (i2c_reg_read_byte(data->i2c_dev, cfg->i2c_addr, REG_INT, &int_reg)) {
            LOG_ERR("Failed to read INT register");
            return -EIO;
        }
        int_reg |= MSK_INT_OUT_EN;
        if (i2c_reg_write_byte(data->i2c_dev, cfg->i2c_addr, REG_INT, int_reg)) {
            LOG_ERR("Failed to enable interrupt output");
            return -EIO;
        }
    } else {
        LOG_ERR("Interrupt GPIO device is not ready");
        return -EINVAL;
    }
#endif

    k_work_init(&data->work, pimoroni_pim447_work_handler);

    /* Optionally, initialize the LED to a default state */
    pimoroni_pim447_led_set(dev, 100, 0, 0, 100); // Turn off the LED initially

    return 0;
}

int pimoroni_pim447_led_set(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t buf[4] = { red, green, blue, white };
    int ret;

    ret = i2c_burst_write(data->i2c_dev, cfg->i2c_addr, REG_LED_RED, buf, 4);
    if (ret) {
        LOG_ERR("Failed to write LED control registers");
        return ret;
    }

    LOG_DBG("Set LED colors: R=%d, G=%d, B=%d, W=%d", red, green, blue, white);

    return 0;
}

int pimoroni_pim447_set_red(const struct device *dev, uint8_t value) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte(data->i2c_dev, cfg->i2c_addr, REG_LED_RED, value);
    if (ret) {
        LOG_ERR("Failed to set red LED value");
    }
    return ret;
}

int pimoroni_pim447_set_green(const struct device *dev, uint8_t value) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte(data->i2c_dev, cfg->i2c_addr, REG_LED_GRN, value);
    if (ret) {
        LOG_ERR("Failed to set green LED value");
    }
    return ret;
}

int pimoroni_pim447_set_blue(const struct device *dev, uint8_t value) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte(data->i2c_dev, cfg->i2c_addr, REG_LED_BLU, value);
    if (ret) {
        LOG_ERR("Failed to set blue LED value");
    }
    return ret;
}

int pimoroni_pim447_set_white(const struct device *dev, uint8_t value) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte(data->i2c_dev, cfg->i2c_addr, REG_LED_WHT, value);
    if (ret) {
        LOG_ERR("Failed to set white LED value");
    }
    return ret;
}

static const struct sensor_driver_api pimoroni_pim447_driver_api = {
    .sample_fetch = pimoroni_pim447_sample_fetch,
    .channel_get = pimoroni_pim447_channel_get,
};

static const struct pimoroni_pim447_config pimoroni_pim447_config = {
    .i2c_bus = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(0))),
    .i2c_addr = PIMORONI_PIM447_I2C_ADDRESS,
#ifdef CONFIG_ZMK_SENSOR_PIMORONI_PIM447_INTERRUPT
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
#endif
};

static struct pimoroni_pim447_data pimoroni_pim447_data;

DEVICE_DT_INST_DEFINE(0,
                     pimoroni_pim447_init,
                     NULL,
                     &pimoroni_pim447_data,
                     &pimoroni_pim447_config,
                     POST_KERNEL,
                     CONFIG_SENSOR_INIT_PRIORITY,
                     &pimoroni_pim447_driver_api);
