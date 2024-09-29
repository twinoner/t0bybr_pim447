/* pimoroni_pim447.c - Driver for Pimoroni PIM447 Trackball */

#define DT_DRV_COMPAT zmk_pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "pimoroni_pim447.h"
#include "pimoroni_pim447_led.h"  // For function declarations


LOG_MODULE_REGISTER(zmk_pimoroni_pim447, LOG_LEVEL_DBG);

#define TRACKBALL_POLL_INTERVAL_MS 8 // Approximately 1/120 second

/* Forward declaration of functions */
static void pimoroni_pim447_periodic_work_handler(struct k_work *work);
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config, bool enable);

/* Periodic work handler function */
static void pimoroni_pim447_periodic_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pimoroni_pim447_data *data = CONTAINER_OF(dwork, struct pimoroni_pim447_data, periodic_work);
    const struct pimoroni_pim447_config *config = data->dev->config;
    const struct device *dev = data->dev;
    int16_t delta_x, delta_y;
    bool sw_pressed;
    int err;

    k_mutex_lock(&data->data_lock, K_FOREVER);


    /* Copy and reset accumulated data */
    delta_x = data->delta_x;
    delta_y = data->delta_y;
    data->delta_x = 0;
    data->delta_y = 0;

    /* Get switch state */
    sw_pressed = data->sw_pressed;

    k_mutex_unlock(&data->data_lock);

    /* Report relative X movement */
    if (delta_x != 0) {
        err = input_report_rel(dev, INPUT_REL_X, delta_x, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_x: %d", err);
        } else {
            LOG_DBG("Reported delta_x: %d", delta_x);
        }
    }

    /* Report relative Y movement */
    if (delta_y != 0) {
        err = input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_y: %d", err);
        } else {
            LOG_DBG("Reported delta_y: %d", delta_y);
        }
    }

    /* Report switch state if it changed */
    if (sw_pressed != data->sw_pressed_prev) {
        err = input_report_key(dev, INPUT_BTN_0, sw_pressed ? 1 : 0, true, K_FOREVER);
        if (err) {
            LOG_ERR("Failed to report switch state: %d", err);
        } else {
            LOG_DBG("Reported switch state: %d", sw_pressed);
            data->sw_pressed_prev = sw_pressed;
        }
    }
    
    /* Read and clear the INT status register if necessary */
    uint8_t int_status;
    int ret;
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

    /* Reschedule the work */
    k_work_schedule(&data->periodic_work, K_MSEC(TRACKBALL_POLL_INTERVAL_MS)); // Schedule next execution
}

/* GPIO callback function */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);
    const struct pimoroni_pim447_config *config = data->dev->config;
    uint8_t buf[5];
    int ret;

    LOG_INF("GPIO interrupt triggered");

    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447");
        return;
    }

    k_mutex_lock(&data->data_lock, K_FOREVER);

    /* Accumulate movement data */
    data->delta_x += (int16_t)buf[1] - (int16_t)buf[0]; // RIGHT - LEFT
    data->delta_y += (int16_t)buf[3] - (int16_t)buf[2]; // DOWN - UP

    /* Update switch state */
    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    k_mutex_unlock(&data->data_lock);

    /* Clear movement registers */
    uint8_t zero = 0;
    i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_UP, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, zero);

    /* Clear the interrupt */
    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret == 0 && (int_status & MSK_INT_TRIGGERED)) {
        int_status &= ~MSK_INT_TRIGGERED;
        i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
    }
}

/* Function to enable or disable interrupt output */
static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config, bool enable) {
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
static int pimoroni_pim447_enable(const struct device *dev) {
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;

    LOG_INF("pimoroni_pim447_enable called");

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
    gpio_init_callback(&data->int_gpio_cb, pimoroni_pim447_gpio_callback, BIT(config->int_gpio.pin));

    /* Add the GPIO callback */
    ret = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback");
        return ret;
    } else {
        LOG_INF("GPIO callback added successfully");
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
    ret = pimoroni_pim447_enable_interrupt(config, true);
    if (ret) {
        LOG_ERR("Failed to enable interrupt output");
        return ret;
    }

    LOG_INF("pimoroni_pim447 enabled");

    return 0;
}

/* Disable function */
static int pimoroni_pim447_disable(const struct device *dev) {
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;

    LOG_INF("pimoroni_pim447_disable called");

    /* Disable interrupt output on the trackball */
    ret = pimoroni_pim447_enable_interrupt(config, false);
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

    LOG_INF("pimoroni_pim447 disabled");

    return 0;
}

static int pimoroni_pim447_test_interrupt(const struct device *dev) {
    const struct pimoroni_pim447_config *config = dev->config;

    printk("Triggering a test interrupt on GPIO %d\n", config->int_gpio.pin);

    // Briefly set the GPIO pin LOW to simulate an interrupt
    gpio_pin_set_dt(&config->int_gpio, 0);  
    k_msleep(10); // Hold low for a short duration
    gpio_pin_set_dt(&config->int_gpio, 1); 

    printk("Test interrupt triggered. Check logs for callback execution.\n");
}

/* Device initialization function */
static int pimoroni_pim447_init(const struct device *dev) {
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;

    LOG_INF("PIM447 driver initializing");

    data->dev = dev;
    data->sw_pressed_prev = false;
    data->delta_x = 0;
    data->delta_y = 0;

    /* Initialize the mutex */
    k_mutex_init(&data->data_lock);
    k_mutex_init(&data->i2c_lock);

    /* Initialize the periodic work handler */
    k_work_init_delayable(&data->periodic_work, pimoroni_pim447_periodic_work_handler);

    /* Start the periodic work */
    k_work_schedule(&data->periodic_work, K_MSEC(TRACKBALL_POLL_INTERVAL_MS));

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
    ret = pimoroni_pim447_enable(dev);
    if (ret) {
        LOG_ERR("Failed to enable PIM447");
        return ret;
    }

    /* Initialize the LED functionality */
    pimoroni_pim447_led_init(dev);

    pimoroni_pim447_test_interrupt(dev);
    
    LOG_INF("PIM447 driver initialized");

    return 0;
}

static const struct pimoroni_pim447_config pimoroni_pim447_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

static struct pimoroni_pim447_data pimoroni_pim447_data;

/* Device initialization macro */
DEVICE_DT_INST_DEFINE(0, pimoroni_pim447_init, NULL, &pimoroni_pim447_data, &pimoroni_pim447_config,
                      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
