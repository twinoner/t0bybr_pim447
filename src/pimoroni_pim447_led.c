/* pimoroni_pim447_led.c */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "pimoroni_pim447.h"      // For shared data structures
#include "pimoroni_pim447_led.h"  // For function declarations

LOG_MODULE_DECLARE(zmk_pimoroni_pim447, LOG_LEVEL_DBG);

/**
 * @brief Set the brightness of an LED.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param led LED to set the brightness for.
 * @param brightness Brightness value (0 to 255).
 *
 * @return 0 if successful, otherwise a negative error code.
 */

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

void hsv_to_rgbw(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w) {
    // Existing HSV to RGB conversion
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

    // Calculate white component
    float w_prime = fminf(r_prime, fminf(g_prime, b_prime));
    r_prime -= w_prime;
    g_prime -= w_prime;
    b_prime -= w_prime;

    // Adjust to 0-255 scale
    *r = (uint8_t)((r_prime + m + w_prime) * 255);
    *g = (uint8_t)((g_prime + m + w_prime) * 255);
    *b = (uint8_t)((b_prime + m + w_prime) * 255);
    *w = (uint8_t)(w_prime * 255);
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

    return 0;
}

void pimoroni_pim447_led_init(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;

}
