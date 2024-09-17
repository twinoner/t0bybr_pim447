#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(pimoroni_pim447, CONFIG_SENSOR_LOG_LEVEL);

#define PIMORONI_PIM447_I2C_ADDRESS 0x0A
#define CHIP_ID 0xBA11  // Replace with actual chip ID if known

/* Register definitions */
#define REG_LEFT       0x04
#define REG_RIGHT      0x05
#define REG_UP         0x06
#define REG_DOWN       0x07
#define REG_SWITCH     0x08
#define REG_LED_RED    0x09
#define REG_LED_GRN    0x0A
#define REG_LED_BLU    0x0B
#define REG_LED_WHT    0x0C
#define REG_CHIP_ID_L  0x00
#define REG_CHIP_ID_H  0x01
#define REG_INT        0x0D

/* Bit masks */
#define MSK_SWITCH_STATE 0x01
#define MSK_INT_OUT_EN   0x01

struct pimoroni_pim447_data {
    const struct device *i2c_dev;
    const struct device *dev;
    int16_t delta_x;
    int16_t delta_y;
    bool sw_pressed;
    struct k_work work;
    struct gpio_callback int_gpio_cb;
};

struct pimoroni_pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

static int pimoroni_pim447_sample_fetch(const struct device *dev) {
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

static void pimoroni_pim447_process_input(struct pimoroni_pim447_data *data) {
    struct zmk_hid_mouse_report mouse_report = {0};

    /* Apply scaling or sensitivity adjustments if needed */
    mouse_report.x = data->delta_x;
    mouse_report.y = data->delta_y;

    /* Update button state */
    if (data->sw_pressed) {
        mouse_report.buttons |= 1;  // Left-click
    }

    /* Send the HID mouse report */
    zmk_hid_mouse_report(&mouse_report);
}

static void pimoroni_pim447_work_handler(struct k_work *work) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, work);

    if (pimoroni_pim447_sample_fetch(data->dev) == 0) {
        pimoroni_pim447_process_input(data);
    }
}

static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);

    k_work_submit(&data->work);
}

static int pimoroni_pim447_init(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t chip_id_l, chip_id_h;
    uint16_t chip_id;
    int ret;

    data->i2c_dev = cfg->i2c.bus;
    if (!device_is_ready(data->i2c_dev)) {
        LOG_ERR("I2C bus device is not ready");
        return -EINVAL;
    }

    data->dev = dev;

    /* Read Chip ID */
    ret = i2c_reg_read_byte_dt(&cfg->i2c, REG_CHIP_ID_L, &chip_id_l);
    if (ret) {
        LOG_ERR("Failed to read chip ID low byte");
        return ret;
    }

    ret = i2c_reg_read_byte_dt(&cfg->i2c, REG_CHIP_ID_H, &chip_id_h);
    if (ret) {
        LOG_ERR("Failed to read chip ID high byte");
        return ret;
    }

    chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    if (chip_id != CHIP_ID) {
        LOG_ERR("Invalid chip ID: 0x%04X", chip_id);
        return -EINVAL;
    }

    LOG_INF("Pimoroni PIM447 detected, chip ID: 0x%04X", chip_id);

    /* Initialize interrupt GPIO */
    if (cfg->int_gpio.port && device_is_ready(cfg->int_gpio.port)) {
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
        ret = i2c_reg_read_byte_dt(&cfg->i2c, REG_INT, &int_reg);
        if (ret) {
            LOG_ERR("Failed to read INT register");
            return ret;
        }
        int_reg |= MSK_INT_OUT_EN;
        ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_INT, int_reg);
        if (ret) {
            LOG_ERR("Failed to enable interrupt output");
            return ret;
        }
    } else {
        LOG_ERR("Interrupt GPIO device is not ready");
        return -EINVAL;
    }

    k_work_init(&data->work, pimoroni_pim447_work_handler);

    /* Initialize the LED to a default state */
    pimoroni_pim447_led_set(dev, 0, 0, 0, 0); // Turn off the LED initially

    return 0;
}

int pimoroni_pim447_led_set(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    uint8_t buf[4] = { red, green, blue, white };
    int ret;

    ret = i2c_burst_write_dt(&cfg->i2c, REG_LED_RED, buf, 4);
    if (ret) {
        LOG_ERR("Failed to write LED control registers");
        return ret;
    }

    LOG_DBG("Set LED colors: R=%d, G=%d, B=%d, W=%d", red, green, blue, white);

    return 0;
}

int pimoroni_pim447_set_red(const struct device *dev, uint8_t value) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_LED_RED, value);
    if (ret) {
        LOG_ERR("Failed to set red LED value");
    }
    return ret;
}

int pimoroni_pim447_set_green(const struct device *dev, uint8_t value) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_LED_GRN, value);
    if (ret) {
        LOG_ERR("Failed to set green LED value");
    }
    return ret;
}

int pimoroni_pim447_set_blue(const struct device *dev, uint8_t value) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_LED_BLU, value);
    if (ret) {
        LOG_ERR("Failed to set blue LED value");
    }
    return ret;
}

int pimoroni_pim447_set_white(const struct device *dev, uint8_t value) {
    const struct pimoroni_pim447_config *cfg = dev->config;
    int ret;

    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_LED_WHT, value);
    if (ret) {
        LOG_ERR("Failed to set white LED value");
    }
    return ret;
}

static const struct device *pim447_dev;

static int pimoroni_pim447_enable(const struct device *dev) {
    /* Enable device if necessary */
    return 0;
}

static int pimoroni_pim447_disable(const struct device *dev) {
    /* Disable device if necessary */
    return 0;
}

static const struct device *pimoroni_pim447_get_device(void) {
    return pim447_dev;
}

static int pimoroni_pim447_driver_init(const struct device *dev) {
    pim447_dev = dev;
    return pimoroni_pim447_init(dev);
}

static const struct pimoroni_pim447_config pimoroni_pim447_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

static struct pimoroni_pim447_data pimoroni_pim447_data;

DEVICE_DT_INST_DEFINE(0,
                      &pimoroni_pim447_driver_init,
                      NULL,
                      &pimoroni_pim447_data,
                      &pimoroni_pim447_config,
                      POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY,
                      NULL);
