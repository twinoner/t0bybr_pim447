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

    /* Enable interrupt output on the trackball */
    ret = pimoroni_pim447_enable_interrupt(config, true);
    if (ret) {
        LOG_ERR("Failed to enable interrupt output");
        return ret;
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

    // Initialize the LED animation work handler
    k_work_init_delayable(&data->led_work, pimoroni_pim447_led_work_handler);
    data->hue = 0.0f;
    data->led_animation_running = false;

    // Optionally start the animation during initialization
    pimoroni_pim447_start_led_animation(dev);

    LOG_INF("PIM447 driver initialized");

    return 0;
}

/* Enumeration for the LEDs */
typedef enum {
    PIM447_LED_RED,
    PIM447_LED_GREEN,
    PIM447_LED_BLUE,
    PIM447_LED_WHITE
} pim447_led_t;

int pimoroni_pim447_set_led(const struct device *dev, pim447_led_t led, uint8_t brightness) {
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    uint8_t reg;
    int ret;

    /* Determine the register based on the specified LED */
    switch (led) {
        case PIM447_LED_RED:
            reg = REG_LED_RED;
            break;
        case PIM447_LED_GREEN:
            reg = REG_LED_GRN;
            break;
        case PIM447_LED_BLUE:
            reg = REG_LED_BLU;
            break;
        case PIM447_LED_WHITE:
            reg = REG_LED_WHT;
            break;
        default:
            LOG_ERR("Invalid LED specified");
            return -EINVAL;
    }

    /* Lock the mutex to ensure thread safety */
    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    /* Write the brightness value to the appropriate LED register */
    ret = i2c_reg_write_byte_dt(&config->i2c, reg, brightness);

    /* Unlock the mutex after the I2C operation */
    k_mutex_unlock(&data->i2c_lock);

    if (ret) {
        LOG_ERR("Failed to set LED brightness: %d", ret);
        return ret;
    }

    LOG_DBG("LED %d brightness set to %d", led, brightness);
    return 0;
}


/**
 * @brief Convert HSV color space to RGB.
 *
 * @param h Hue angle in degrees (0 to 360).
 * @param s Saturation (0 to 1).
 * @param v Value/Brightness (0 to 1).
 * @param r Pointer to store Red component (0 to 255).
 * @param g Pointer to store Green component (0 to 255).
 * @param b Pointer to store Blue component (0 to 255).
 */
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    float r_prime, g_prime, b_prime;

    if (h < 60) {
        r_prime = c;
        g_prime = x;
        b_prime = 0;
    } else if (h < 120) {
        r_prime = x;
        g_prime = c;
        b_prime = 0;
    } else if (h < 180) {
        r_prime = 0;
        g_prime = c;
        b_prime = x;
    } else if (h < 240) {
        r_prime = 0;
        g_prime = x;
        b_prime = c;
    } else if (h < 300) {
        r_prime = x;
        g_prime = 0;
        b_prime = c;
    } else {
        r_prime = c;
        g_prime = 0;
        b_prime = x;
    }

    *r = (uint8_t)((r_prime + m) * 255);
    *g = (uint8_t)((g_prime + m) * 255);
    *b = (uint8_t)((b_prime + m) * 255);
}

static void pimoroni_pim447_led_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pimoroni_pim447_data *data = CONTAINER_OF(dwork, struct pimoroni_pim447_data, led_work);
    const struct device *dev = data->dev;
    uint8_t r, g, b;
    int ret;

    // Increment hue
    data->hue += 1.0f;
    if (data->hue >= 360.0f) {
        data->hue -= 360.0f;
    }

    // Convert HSV to RGB
    hsv_to_rgb(data->hue, 1.0f, 1.0f, &r, &g, &b);

    // Set the RGB LEDs
    ret = pimoroni_pim447_set_leds(dev, r, g, b, 0);
    if (ret) {
        LOG_ERR("Failed to set LEDs during animation: %d", ret);
    }

    // Reschedule the work if animation is running
    if (data->led_animation_running) {
        k_work_schedule(&data->led_work, K_MSEC(LED_ANIMATION_INTERVAL_MS));
    }
}

/**
 * @brief Start the LED color cycling animation.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
void pimoroni_pim447_start_led_animation(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;

    if (data->led_animation_running) {
        return; // Animation already running
    }

    data->led_animation_running = true;
    data->hue = 0.0f; // Start from hue 0
    k_work_schedule(&data->led_work, K_NO_WAIT);
}

/**
 * @brief Stop the LED color cycling animation.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
void pimoroni_pim447_stop_led_animation(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;

    data->led_animation_running = false;
    k_work_cancel_delayable(&data->led_work);

    // Optionally turn off the LEDs
    pimoroni_pim447_set_leds(dev, 0, 0, 0, 0);
}

int pimoroni_pim447_set_leds(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;
    uint8_t led_values[4];

    led_values[0] = red;
    led_values[1] = green;
    led_values[2] = blue;
    led_values[3] = white;

    /* Lock the mutex to ensure thread safety */
    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    /* Perform a burst write to set all LED brightness values at once */
    ret = i2c_burst_write_dt(&config->i2c, REG_LED_RED, led_values, sizeof(led_values));

    /* Unlock the mutex after the I2C operation */
    k_mutex_unlock(&data->i2c_lock);

    if (ret) {
        LOG_ERR("Failed to set LED brightness levels: %d", ret);
        return ret;
    }

    LOG_DBG("LED brightness set to R:%d, G:%d, B:%d, W:%d", red, green, blue, white);
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
