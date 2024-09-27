/* pim447.c - Driver for Pimoroni PIM447 Trackball */

#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <math.h>


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

#define ACCUMULATION_THRESHOLD 30
#define BASE_SCALE_FACTOR 3.0f
#define EXPONENTIAL_FACTOR 10.0f
#define SMOOTHING_FACTOR 0.7f
#define REPORT_INTERVAL_MS 10

/* Exposed variables */
volatile float speed_min = 1.0f;
volatile float speed_max = 5.0f;
volatile float scale_divisor_min = 1.0f;
volatile float scale_divisor_max = 2.0f;

/* Mutex for thread safety */
K_MUTEX_DEFINE(variable_mutex);

LOG_MODULE_REGISTER(pim447, LOG_LEVEL_DBG);




/* Device configuration structure */
struct pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

/* Device data structure */
struct pim447_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback int_gpio_cb;
    bool sw_pressed_prev;
    int32_t accum_x;
    int32_t accum_y;
    float smooth_x;
    float smooth_y;
    uint32_t last_report_time;
};

/* Forward declaration of functions */
static void pim447_work_handler(struct k_work *work);
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pim447_enable_interrupt(const struct pim447_config *config, bool enable);

static float apply_exponential_scaling(float value) {
    float abs_value = fabsf(value);
    float scaled = BASE_SCALE_FACTOR * powf(abs_value, EXPONENTIAL_FACTOR);
    return (value >= 0) ? scaled : -scaled;
}

/* Work handler function */
static void pim447_work_handler(struct k_work *work) {
    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work);
    const struct pim447_config *config = data->dev->config;
    const struct device *dev = data->dev;

    uint8_t buf[5];
    int ret;

    LOG_INF("Work handler executed");

    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447");
        return;
    }

    LOG_INF("Raw data: LEFT=%d, RIGHT=%d, UP=%d, DOWN=%d, SWITCH=0x%02X",
            buf[0], buf[1], buf[2], buf[3], buf[4]);

    // Calculate movement deltas
    float delta_x = (float)((int8_t)buf[1] - (int8_t)buf[0]); // Right - Left
    float delta_y = (float)((int8_t)buf[3] - (int8_t)buf[2]); // Down - Up

    // Accumulate deltas
    data->accum_x += delta_x;
    data->accum_y += delta_y;

    uint32_t current_time = k_uptime_get_32();
    uint32_t time_diff = current_time - data->last_report_time;

    // Check if we should report movement
    if (fabsf(data->accum_x) >= ACCUMULATION_THRESHOLD || 
        fabsf(data->accum_y) >= ACCUMULATION_THRESHOLD ||
        time_diff >= REPORT_INTERVAL_MS) {

        // Apply exponential scaling
        float scaled_x = apply_exponential_scaling(data->accum_x);
        float scaled_y = apply_exponential_scaling(data->accum_y);

        // Apply smoothing
        data->smooth_x = (SMOOTHING_FACTOR * data->smooth_x) + ((1 - SMOOTHING_FACTOR) * scaled_x);
        data->smooth_y = (SMOOTHING_FACTOR * data->smooth_y) + ((1 - SMOOTHING_FACTOR) * scaled_y);

        int16_t report_x = (int16_t)roundf(data->smooth_x);
        int16_t report_y = (int16_t)roundf(data->smooth_y);

        LOG_INF("Reporting movement: delta_x=%d, delta_y=%d", report_x, report_y);

        int err;

        /* Report relative X movement */
        err = input_report_rel(dev, INPUT_REL_X, report_x, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_x: %d", err);
        } else {
            LOG_DBG("Reported delta_x: %d", report_x);
        }

        /* Report relative Y movement */
        err = input_report_rel(dev, INPUT_REL_Y, report_y, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_y: %d", err);
        } else {
            LOG_DBG("Reported delta_y: %d", report_y);
        }

        // Reset accumulation
        data->accum_x = 0;
        data->accum_y = 0;
        data->last_report_time = current_time;
    }


    bool sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    int err;


    err = input_report_key(dev, INPUT_BTN_0, sw_pressed ? 1 : 0, true, K_FOREVER);
    if (err) {
        LOG_ERR("Failed to report switch state: %d", err);
    } else {
        LOG_DBG("Reported switch state: %d", sw_pressed);
    }


    /* Clear movement registers by writing zeros */
    uint8_t zero = 0;
    ret = i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_UP, zero);
    ret |= i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, zero);

    if (ret) {
        LOG_ERR("Failed to clear movement registers");
    }

    /* Log the movement data */
    if (delta_x || delta_y || sw_pressed != data->sw_pressed_prev) {
        LOG_INF("Trackball moved: delta_x=%d, delta_y=%d, sw_pressed=%d",
                delta_x, delta_y, sw_pressed);
        data->sw_pressed_prev = sw_pressed;
    }


    

    /* Read and clear the INT status register if necessary */
    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret) {
        LOG_ERR("Failed to read INT status register");
        return;
    }

    LOG_INF("INT status before clearing: 0x%02X", int_status);

    if (int_status & MSK_INT_TRIGGERED) {
        int_status &= ~MSK_INT_TRIGGERED;
        ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
        if (ret) {
            LOG_ERR("Failed to clear INT status register");
            return;
        }
        LOG_INF("INT status after clearing: 0x%02X", int_status);
    }
}

/* GPIO callback function */
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_gpio_cb);
    const struct pim447_config *config = data->dev->config;

    LOG_INF("GPIO callback triggered on pin %d", config->int_gpio.pin);

    k_work_submit(&data->work);
}

/* Function to enable or disable interrupt output */
static int pim447_enable_interrupt(const struct pim447_config *config, bool enable) {
    uint8_t int_reg;
    int ret;

    /* Read the current INT register value */
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_reg);
    if (ret) {
        LOG_ERR("Failed to read INT register");
        return ret;
    }

    LOG_INF("INT register before changing: 0x%02X", int_reg);

    /* Update the MSK_INT_OUT_EN bit */
    if (enable) {
        int_reg |= MSK_INT_OUT_EN;
    } else {
        int_reg &= ~MSK_INT_OUT_EN;
    }

    /* Write the updated INT register value */
    ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_reg);
    if (ret) {
        LOG_ERR("Failed to write INT register");
        return ret;
    }

    LOG_INF("INT register after changing: 0x%02X", int_reg);

    return 0;
}

/* Enable function */
static int pim447_enable(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    LOG_INF("pim447_enable called");

    /* Check if the interrupt GPIO device is ready */
    if (!device_is_ready(config->int_gpio.port)) {
        LOG_ERR("Interrupt GPIO device is not ready");
        return -ENODEV;
    }

    /* Configure the interrupt GPIO pin without internal pull-up (external pull-up used) */
    ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Failed to configure interrupt GPIO");
        return ret;
    }

    /* Initialize the GPIO callback */
    gpio_init_callback(&data->int_gpio_cb, pim447_gpio_callback, BIT(config->int_gpio.pin));

    /* Add the GPIO callback */
    ret = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback");
        return ret;
    }

    /* Configure the GPIO interrupt for falling edge (active low) */
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_FALLING);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt");
        return ret;
    }

    /* Clear any pending interrupts */
    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret) {
        LOG_ERR("Failed to read INT status register");
        return ret;
    }

    /* Clear the MSK_INT_TRIGGERED bit */
    int_status &= ~MSK_INT_TRIGGERED;
    ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
    if (ret) {
        LOG_ERR("Failed to clear INT status register");
        return ret;
    }

    /* Enable interrupt output on the trackball */
    ret = pim447_enable_interrupt(config, true);
    if (ret) {
        LOG_ERR("Failed to enable interrupt output");
        return ret;
    }

    LOG_INF("pim447 enabled");

    return 0;
}

/* Disable function */
static int pim447_disable(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    LOG_INF("pim447_disable called");

    /* Disable interrupt output on the trackball */
    ret = pim447_enable_interrupt(config, false);
    if (ret) {
        LOG_ERR("Failed to disable interrupt output");
        return ret;
    }

    /* Disable GPIO interrupt */
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
    if (ret) {
        LOG_ERR("Failed to disable GPIO interrupt");
        return ret;
    }

    /* Remove the GPIO callback */
    gpio_remove_callback(config->int_gpio.port, &data->int_gpio_cb);

    LOG_INF("pim447 disabled");

    return 0;
}

/* Device initialization function */
static int pim447_init(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    data->dev = dev;
    data->sw_pressed_prev = false;
    data->accum_x = 0;
    data->accum_y = 0;
    data->smooth_x = 0;
    data->smooth_y = 0;
    data->last_report_time = k_uptime_get_32();

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

    /* Enable the Trackball */
    pim447_enable(dev);

    /* Initialize the work handler */
    k_work_init(&data->work, pim447_work_handler);

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
                        POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
