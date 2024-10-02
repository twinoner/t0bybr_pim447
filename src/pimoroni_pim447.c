/* pimoroni_pim447.c - Driver for Pimoroni PIM447 Trackball */

#define DT_DRV_COMPAT zmk_pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/activity_state_changed.h>
#include <math.h>

#include "pimoroni_pim447.h"
#include "pimoroni_pim447_led.h"  // For function declarations

LOG_MODULE_REGISTER(zmk_pimoroni_pim447, LOG_LEVEL_DBG);

volatile uint8_t PIM447_MOUSE_MAX_SPEED = 25;
volatile uint8_t PIM447_MOUSE_MAX_TIME = 5;
volatile float PIM447_MOUSE_SMOOTHING_FACTOR = 1.3f;
volatile uint8_t PIM447_SCROLL_MAX_SPEED = 1;
volatile uint8_t PIM447_SCROLL_MAX_TIME = 1;
volatile float PIM447_SCROLL_SMOOTHING_FACTOR = 0.5f;
volatile float PIM447_HUE_INCREMENT_FACTOR = 0.3f;

enum pim447_mode {
    PIM447_MODE_MOUSE,
    PIM447_MODE_SCROLL
};

static enum pim447_mode current_mode = PIM447_MODE_MOUSE;

/* Forward declaration of functions */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config, bool enable);
static void activate_automouse_layer();
static void deactivate_automouse_layer(struct k_timer *timer);

static int previous_x = 0;
static int previous_y = 0;

void pim447_enable_sleep(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;

    const struct pimoroni_pim447_config *config = data->dev->config;
    uint8_t ctrl_reg_value;

    // Read the current control register value
    if (i2c_reg_read_byte_dt(&config->i2c, MSK_CTRL_SLEEP, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read PIM447 control register");
        return;
    }

    ctrl_reg_value |= MSK_CTRL_SLEEP; // Set the SLEEP bit

    // Write the modified value back
    if (i2c_reg_write_byte_dt(&config->i2c, MSK_CTRL_SLEEP, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write PIM447 control register");
        return;
    }

    pimoroni_pim447_set_leds(dev, 0, 0, 0, 0); // Turn off LEDs

    LOG_DBG("PIM447 sleep enabled"); 
}

void pim447_disable_sleep(const struct device *dev) {
    struct pimoroni_pim447_data *data = dev->data;

    const struct pimoroni_pim447_config *config = data->dev->config;
    uint8_t ctrl_reg_value;

    // Read the current control register value
    if (i2c_reg_read_byte_dt(&config->i2c, MSK_CTRL_SLEEP, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read PIM447 control register");
        return;
    }

    ctrl_reg_value &= ~MSK_CTRL_SLEEP; // Clear the SLEEP bit

    // Write the modified value back
    if (i2c_reg_write_byte_dt(&config->i2c, MSK_CTRL_SLEEP, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write PIM447 control register");
        return;
    }

    pimoroni_pim447_set_leds(dev, 255, 0, 0, 0); // Turn on LEDs

    LOG_DBG("PIM447 sleep disabled");
}

void pim447_toggle_mode(void) {
    current_mode = (current_mode == PIM447_MODE_MOUSE) ? PIM447_MODE_SCROLL : PIM447_MODE_MOUSE;
    // Optional: Add logging or LED indication here to show the current mode
    LOG_DBG("PIM447 mode switched to %s", (current_mode == PIM447_MODE_MOUSE) ? "MOUSE" : "SCROLL");
}

// Event handler for activity state changes
static int activity_state_changed_handler(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    // Get the device pointer
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(pimoroni_pim447));
    if (!device_is_ready(dev)) {
        LOG_ERR("PIM447 device not ready");
        return -ENODEV;
    }

    if (ev->state == ZMK_ACTIVITY_IDLE) {
        pim447_enable_sleep(dev);
    }

    if (ev->state != ZMK_ACTIVITY_IDLE) {
        pim447_disable_sleep(dev);
    }

    return 0;
}

ZMK_LISTENER(idle_listener, activity_state_changed_handler);
ZMK_SUBSCRIPTION(idle_listener, zmk_activity_state_changed);

static void pim447_process_movement(struct pimoroni_pim447_data *data, int delta_x, int delta_y, uint32_t time_between_interrupts, int max_speed, int max_time, float smoothing_factor) {
    float scaling_factor = 1.0f;
    if (time_between_interrupts < max_time) {
        // Exponential scaling calculation
        float exponent = -3.0f * (float)time_between_interrupts / max_time; // Adjust -3.0f for desired curve
        scaling_factor = 1.0f + (max_speed - 1.0f) * expf(exponent); 
    }

    // Apply scaling based on mode
    if (current_mode == PIM447_MODE_SCROLL) {
        scaling_factor *= 2.5f; // Example: Increase scaling for scroll mode
    }

    /* Accumulate deltas atomically */
    atomic_add(&data->x_buffer, delta_x);
    atomic_add(&data->y_buffer, delta_y);

    int scaled_x_movement = (int)(delta_x * scaling_factor);
    int scaled_y_movement = (int)(delta_y * scaling_factor);

    // Apply smoothing
    data->smoothed_x = (int)(smoothing_factor * scaled_x_movement + (1.0f - smoothing_factor) * previous_x);
    data->smoothed_y = (int)(smoothing_factor * scaled_y_movement + (1.0f - smoothing_factor) * previous_y);

    data->previous_x = data->smoothed_x;
    data->previous_y = data->smoothed_y;
}

static void pimoroni_pim447_work_handler(struct k_work *work) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, irq_work);
    const struct pimoroni_pim447_config *config = data->dev->config;
    const struct device *dev = data->dev;
    uint8_t buf[5];
    int ret;

    LOG_INF("PIM447 work handler triggered");

 
    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447: %d", ret);
        return;
    }

    uint32_t time_between_interrupts;

    k_mutex_lock(&data->data_lock, K_FOREVER);
    time_between_interrupts = data->last_interrupt_time - data->previous_interrupt_time;
    k_mutex_unlock(&data->data_lock);

    /* Calculate deltas */
    int16_t delta_x = (int16_t)buf[1] - (int16_t)buf[0]; // RIGHT - LEFT
    int16_t delta_y = (int16_t)buf[3] - (int16_t)buf[2]; // DOWN - UP

    /* Report movement immediately if non-zero */
    if (delta_x != 0 || delta_y != 0) {
        if (current_mode == PIM447_MODE_MOUSE) {
            pim447_process_movement(data, delta_x, delta_y, time_between_interrupts, PIM447_MOUSE_MAX_SPEED, PIM447_MOUSE_MAX_TIME, PIM447_MOUSE_SMOOTHING_FACTOR); 
            
            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_X, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_Y, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        } else if (current_mode == PIM447_MODE_SCROLL) {
            pim447_process_movement(data, delta_x, delta_y, time_between_interrupts, PIM447_SCROLL_MAX_SPEED, PIM447_SCROLL_MAX_TIME, PIM447_SCROLL_SMOOTHING_FACTOR); 
            
            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_WHEEL, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_HWHEEL, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        }
    }

    /* Update switch state */
    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    /* Report switch state if it changed */
    if (data->sw_pressed != data->sw_pressed_prev) {
        ret = input_report_key(data->dev, INPUT_BTN_0, data->sw_pressed ? 1 : 0, true, K_NO_WAIT);
        if (ret) {
            LOG_ERR("Failed to report key");
        } else {
            LOG_DBG("Reported key");
        }

        LOG_DBG("Reported switch state: %d", data->sw_pressed);

        data->sw_pressed_prev = data->sw_pressed;
    }

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

        float speed = 0.0f;

        if (delta_x > 0 ||  delta_y > 0) {
            // Calculate movement speed
            speed = sqrtf((float)(delta_x * delta_x + delta_y * delta_y));
        }

    // Update LEDs based on movement
    if (speed > 0) {
         activate_automouse_layer();

            // Update hue or brightness based on speed
            data->hue += speed * PIM447_HUE_INCREMENT_FACTOR;
            if (data->hue >= 360.0f) {
                data->hue -= 360.0f;
            }

         // Convert HSV to RGBW
         uint8_t r, g, b, w;
         hsv_to_rgbw(data->hue, 1.0f, 1.0f, &r, &g, &b, &w);

        int err;

         // Set the LEDs
         err = pimoroni_pim447_set_leds(dev, r, g, b, w);
         if (err) {
             LOG_ERR("Failed to set LEDs: %d", err);
         }
    }
}

/* GPIO callback function */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);

    uint32_t current_time = k_uptime_get();

    k_mutex_lock(&data->data_lock, K_NO_WAIT);
    data->previous_interrupt_time = data->last_interrupt_time;
    data->last_interrupt_time = current_time;
    k_mutex_unlock(&data->data_lock);

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

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
    struct k_timer automouse_layer_timer;
    static bool automouse_triggered = false;

    static void activate_automouse_layer() {
        automouse_triggered = true;
        zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
        k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_ZMK_PIMORONI_PIM447_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
    }

    static void deactivate_automouse_layer(struct k_timer *timer) {
        automouse_triggered = false;
        zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
    }

    K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

static const struct pimoroni_pim447_config pimoroni_pim447_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

static struct pimoroni_pim447_data pimoroni_pim447_data;

/* Device initialization macro */
DEVICE_DT_INST_DEFINE(0, pimoroni_pim447_init, NULL, &pimoroni_pim447_data, &pimoroni_pim447_config,
                      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
