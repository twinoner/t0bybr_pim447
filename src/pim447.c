#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


#include <zmk/hid.h>
// #include <zmk/mouse.h>
// #include <zmk/endpoints.h>

#define MOUSE_BUTTON_LEFT 0x01

/* Register Addresses */
#define REG_LED_RED     0x00
#define REG_LED_GRN     0x01
#define REG_LED_BLU     0x02
#define REG_LED_WHT     0x03
#define REG_LEFT        0x04
#define REG_RIGHT       0x05
#define REG_UP          0x06
#define REG_DOWN        0x07
#define REG_SWITCH      0x08
#define REG_USER_FLASH  0xD0
#define REG_FLASH_PAGE  0xF0
#define REG_INT         0xF9
#define REG_CHIP_ID_L   0xFA
#define REG_CHIP_ID_H   0xFB
#define REG_VERSION     0xFC
#define REG_I2C_ADDR    0xFD
#define REG_CTRL        0xFE

/* Bit Masks */
#define MSK_SWITCH_STATE    0b10000000

/* Interrupt Masks */
#define MSK_INT_TRIGGERED   0b00000001
#define MSK_INT_OUT_EN      0b00000010

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct pim447_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback int_gpio_cb;
    bool sw_pressed_prev;
};

static void pim447_work_handler(struct k_work *work);

static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                 gpio_port_pins_t pins) {
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_gpio_cb);
    k_work_submit(&data->work);
}

/* Work handler function that reads movement data and sends HID reports */
static void pim447_work_handler(struct k_work *work) {
    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work);
    const struct pim447_config *config = data->dev->config;
    uint8_t buf[5];
    int ret;

    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447");
        return;
    }

    /* Calculate movement deltas */
    int16_t delta_x = (int16_t)buf[1] - (int16_t)buf[0]; // Right - Left
    int16_t delta_y = (int16_t)buf[3] - (int16_t)buf[2]; // Down - Up
    bool sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    /* Clear movement registers by writing zeros */
    uint8_t zero = 0;
    ret = i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_UP, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, zero);

    if (ret) {
        LOG_ERR("Failed to clear movement registers");
    }

    /* Send HID mouse report if there's movement or button state changed */
    if (delta_x || delta_y || sw_pressed != data->sw_pressed_prev) {
        /* Get the current mouse report */
        struct zmk_hid_mouse_report *mouse_report = zmk_hid_get_mouse_report();

        /* Update the mouse report */
        mouse_report->x = delta_x;
        mouse_report->y = delta_y;

        /* Handle button state */
        if (sw_pressed) {
            mouse_report->buttons |= MOUSE_BUTTON_LEFT;
        } else {
            mouse_report->buttons &= ~MOUSE_BUTTON_LEFT;
        }

        /* Send the HID report */
        int err = zmk_endpoints_send_mouse_report(mouse_report);
        if (err) {
            LOG_ERR("Failed to send HID mouse report: %d", err);
        }

        /* Update previous button state */
        data->sw_pressed_prev = sw_pressed;

        /* Log the movement data */
        LOG_INF("Trackball moved: delta_x=%d, delta_y=%d, sw_pressed=%d", delta_x, delta_y, sw_pressed);
    }

    /* Read and clear the INT status register if necessary */
    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret) {
        LOG_ERR("Failed to read INT status register");
        return;
    }

    if (int_status & MSK_INT_TRIGGERED) {
        int_status &= ~MSK_INT_TRIGGERED;
        ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
        if (ret) {
            LOG_ERR("Failed to clear INT status register");
            return;
        }
    }
}

static int pim447_init(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;
    uint8_t int_reg;

    data->dev = dev;
    data->sw_pressed_prev = false;

    /* Check if the I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    /* Read and log the chip ID */
    uint8_t chip_id_l, chip_id_h;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_L, &chip_id_l);
    if (ret) {
        LOG_ERR("Failed to read chip ID low byte");
        return ret;
    }

    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_H, &chip_id_h);
    if (ret) {
        LOG_ERR("Failed to read chip ID high byte");
        return ret;
    }

    uint16_t chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    LOG_INF("PIM447 chip ID: 0x%04X", chip_id);

    /* Initialize the work handler */
    k_work_init(&data->work, pim447_work_handler);

    /* Check if the interrupt GPIO device is ready */
    if (!device_is_ready(config->int_gpio.port)) {
        LOG_ERR("Interrupt GPIO device is not ready");
        return -ENODEV;
    }

    /* Configure the interrupt GPIO pin */
    ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure interrupt GPIO");
        return ret;
    }

    /* Initialize the GPIO callback */
    gpio_init_callback(&data->int_gpio_cb, pim447_gpio_callback,
                       BIT(config->int_gpio.pin));

    /* Add the GPIO callback */
    ret = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback");
        return ret;
    }

    /* Configure the GPIO interrupt */
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt");
        return ret;
    }

    /* Enable interrupt output on the trackball */
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_reg);
    if (ret) {
        LOG_ERR("Failed to read INT register");
        return ret;
    }

    LOG_INF("INT register before enabling interrupt: 0x%02X", int_reg);

    int_reg |= MSK_INT_OUT_EN;
    ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_reg);
    if (ret) {
        LOG_ERR("Failed to enable interrupt output");
        return ret;
    }

    /* Read back the INT register to confirm */
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_reg);
    if (ret) {
        LOG_ERR("Failed to read INT register after enabling");
        return ret;
    }
    LOG_INF("INT register after enabling interrupt: 0x%02X", int_reg);

    LOG_INF("PIM447 driver initialized");

    return 0;
}

/* Device configuration */
static const struct pim447_config pim447_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

/* Device data */
static struct pim447_data pim447_data;

/* Device initialization macro */
DEVICE_DT_INST_DEFINE(0, pim447_init, NULL, &pim447_data, &pim447_config,
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
