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

#define TRACKBALL_POLL_INTERVAL_MS 20 // Approximately 1/50 second
#define HUE_INCREMENT_FACTOR 1.0f

/* Forward declaration of functions */
static void pimoroni_pim447_periodic_work_handler(struct k_work *work);
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config, bool enable);

static int16_t convert_speed(int16_t value)
{
    bool negative = (value < 0);

    if (negative) {
        value = -value;
    }

    switch (value) {
        case 0:  value = 0;   break;
        case 1:  value = 1;   break;
        case 2:  value = 4;   break;
        case 3:  value = 8;   break;
        case 4:  value = 18;  break;
        case 5:  value = 32;  break;
        case 6:  value = 50;  break;
        case 7:  value = 72;  break;
        case 8:  value = 98;  break;
        default: value = 127; break;
    }

    if (negative) {
        value = -value;
    }

    return value;
}

/* Periodic work handler function */
static void pimoroni_pim447_periodic_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pimoroni_pim447_data *data = CONTAINER_OF(dwork, struct pimoroni_pim447_data, periodic_work);
    const struct device *dev = data->dev;
    int16_t delta_x, delta_y;
    bool sw_pressed;
    int err;

    k_mutex_lock(&data->data_lock, K_NO_WAIT);

    /* Copy and reset accumulated data */
    delta_x = data->delta_x;
    delta_y = data->delta_y;
    data->delta_x = 0;
    data->delta_y = 0;

    /* Get switch state */
    sw_pressed = data->sw_pressed;

    delta_x = convert_speed(delta_x);
    delta_y = convert_speed(delta_y);

    k_mutex_unlock(&data->data_lock);

    float speed = 0.0f;

    if (delta_x > 0 ||  delta_y > 0) {
        // Calculate movement speed
        speed = sqrtf((float)(delta_x * delta_x + delta_y * delta_y));
    }

    /* Report relative X movement */
    if (delta_x != 0) {
        err = input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_x: %d", err);
        } else {
            LOG_DBG("Reported delta_x: %d", delta_x);

        }
    }

    /* Report relative Y movement */
    if (delta_y != 0) {
        err = input_report_rel(dev, INPUT_REL_Y, delta_y, false, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_y: %d", err);
        } else {
            LOG_DBG("Reported delta_y: %d", delta_y);
        }
    }

    /* Report switch state if it changed */
    if (sw_pressed != data->sw_pressed_prev) {
        err = input_report_key(dev, INPUT_BTN_0, sw_pressed ? 1 : 0, false, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report switch state: %d", err);
        } else {
            LOG_DBG("Reported switch state: %d", sw_pressed);
            data->sw_pressed_prev = sw_pressed;
        }
    }

    // Update LEDs based on movement
    if (speed > 0) {
        // Update hue or brightness based on speed
        data->hue += speed * HUE_INCREMENT_FACTOR;
        if (data->hue >= 360.0f) {
            data->hue -= 360.0f;
        }

        // Convert HSV to RGBW
        uint8_t r, g, b, w;
        hsv_to_rgbw(data->hue, 1.0f, 1.0f, &r, &g, &b, &w);

        // Set the LEDs
        err = pimoroni_pim447_set_leds(dev, r, g, b, w);
        if (err) {
            LOG_ERR("Failed to set LEDs: %d", err);
        }
    }

    /* Reschedule the work */
    k_work_schedule(&data->periodic_work, K_MSEC(TRACKBALL_POLL_INTERVAL_MS)); // Schedule next execution
}

static void pimoroni_pim447_work_handler(struct k_work *work) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, irq_work);
    const struct pimoroni_pim447_config *config = data->dev->config;
    uint8_t buf[5];
    int ret;

    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447: %d", ret);
        return;
    }

    LOG_INF("PIM447 work handler triggered");

    k_mutex_lock(&data->data_lock, K_NO_WAIT);

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


/* GPIO callback function */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);

    /* Schedule the work item to handle the interrupt in thread context */
    k_work_submit(&data->irq_work);
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

    /* Configure the GPIO interrupt for falling edge (active low) */
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_FALLING);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt");
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

    // /* Clear any pending interrupts */
    // uint8_t int_status;
    // ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    // if (ret) {
    //     LOG_ERR("Failed to read INT status register");
    //     return ret;
    // }

    // /* Clear the MSK_INT_TRIGGERED bit */
    // int_status &= ~MSK_INT_TRIGGERED;
    // ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
    // if (ret) {
    //     LOG_ERR("Failed to clear INT status register");
    //     return ret;
    // }
    
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

    k_work_init(&data->irq_work, pimoroni_pim447_work_handler);
    
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
