#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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

/* Control Masks */
#define MSK_CTRL_SLEEP      0b00000001
#define MSK_CTRL_RESET      0b00000010
#define MSK_CTRL_FREAD      0b00000100
#define MSK_CTRL_FWRITE     0b00001000

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct pim447_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback int_gpio_cb;
};

/* Work handler function that reads movement data */
static void pim447_work_handler(struct k_work *work) {
    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work);
    const struct pim447_config *config = data->dev->config;
    uint8_t buf[5];
    int ret;

    LOG_INF("Work handler executed");

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

    LOG_INF("Trackball moved: delta_x=%d, delta_y=%d, sw_pressed=%d", delta_x, delta_y, sw_pressed);

    /* Read and clear the INT status register */
    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret) {
        LOG_ERR("Failed to read INT status register");
        return;
    }

    LOG_DBG("INT status before clearing: 0x%02X", int_status);

    /* Clear the interrupt triggered bit if necessary */
    if (int_status & MSK_INT_TRIGGERED) {
        int_status &= ~MSK_INT_TRIGGERED;
        ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
        if (ret) {
            LOG_ERR("Failed to clear INT status register");
            return;
        }
        LOG_DBG("INT status after clearing: 0x%02X", int_status);
    }

    /* Optionally, check the interrupt pin state */
    int int_pin_state = gpio_pin_get_dt(&config->int_gpio);
    LOG_DBG("Interrupt GPIO pin state after handling: %d", int_pin_state);
}

/* GPIO callback function triggered by the interrupt */
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                 gpio_port_pins_t pins) {
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_gpio_cb);

    LOG_DBG("Interrupt detected on GPIO pin");

    /* Schedule the work handler to process the movement */
    k_work_submit(&data->work);
}

static int pim447_init(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;
    uint8_t int_reg;

    data->dev = dev;

    /* Check if the I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

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
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_BOTH);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt");
        return ret;
    }

    /* Initialize the work handler */
    k_work_init(&data->work, pim447_work_handler);

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
