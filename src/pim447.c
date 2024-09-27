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

#define MOVEMENT_HISTORY_SIZE 5


/* Exposed variables */
volatile uint8_t FREQUENCY_THRESHOLD = 100;
volatile float BASE_SCALE_FACTOR = 1.0f;
volatile float MAX_SCALE_FACTOR = 5.0f;
volatile float SMOOTHING_FACTOR = 0.2f;
volatile uint8_t INTERPOLATION_STEPS = 5;
volatile float EXPONENTIAL_BASE = 1.5f;
volatile float DIAGONAL_THRESHOLD = 0.7f;
volatile float DIAGONAL_BOOST = 1.2f;

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
};

struct movement_data {
    int8_t delta_x;
    int8_t delta_y;
    uint32_t timestamp;
};

static struct movement_data movement_history[MOVEMENT_HISTORY_SIZE];
static int history_index = 0;

static float smooth_value(float current, float target, float factor) {
    return current + factor * (target - current);
}

static float apply_exponential_scaling(float value, float base) {
    float sign = (value >= 0) ? 1 : -1;
    return sign * (powf(base, fabsf(value)) - 1);
}

static void interpolate_movement(float start_x, float start_y, float end_x, float end_y, int steps, const struct device *dev) {
    float step_x = (end_x - start_x) / steps;
    float step_y = (end_y - start_y) / steps;

    for (int i = 1; i <= steps; i++) {
        float interp_x = start_x + i * step_x;
        float interp_y = start_y + i * step_y;

        input_report_rel(dev, INPUT_REL_X, (int)interp_x, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, (int)interp_y, (i == steps), K_NO_WAIT);

               int err;

        /* Report relative X movement */
        err = input_report_rel(dev, INPUT_REL_X, interp_x, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_x: %d", err);
        } else {
            LOG_DBG("Reported delta_x: %d", interp_x);
        }

        /* Report relative Y movement */
        err = input_report_rel(dev, INPUT_REL_Y, interp_y, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report delta_y: %d", err);
        } else {
            LOG_DBG("Reported delta_y: %d", interp_y);
        }
    }
}

static float calculate_frequency_scale(const struct movement_data *history) {
    if (history[0].timestamp == history[MOVEMENT_HISTORY_SIZE - 1].timestamp) {
        return BASE_SCALE_FACTOR;  // Avoid division by zero
    }

    uint32_t time_span = history[0].timestamp - history[MOVEMENT_HISTORY_SIZE - 1].timestamp;
    float movements_per_second = (float)(MOVEMENT_HISTORY_SIZE - 1) * 1000.0f / time_span;

    float scale = BASE_SCALE_FACTOR * (1.0f + (movements_per_second / FREQUENCY_THRESHOLD));
    return MIN(scale, MAX_SCALE_FACTOR);
}

static void process_movement(const struct device *dev, float delta_x, float delta_y) {
    // Calculate the magnitude of the movement
    float magnitude = sqrtf(delta_x * delta_x + delta_y * delta_y);

    // Normalize the movement
    float norm_x = delta_x / magnitude;
    float norm_y = delta_y / magnitude;

    // Check if the movement is diagonal
    k_mutex_lock(&variable_mutex, K_FOREVER);
    bool is_diagonal = (fabsf(norm_x) > DIAGONAL_THRESHOLD) && (fabsf(norm_y) > DIAGONAL_THRESHOLD);
    k_mutex_unlock(&variable_mutex);

    // Apply smoothing
    k_mutex_lock(&variable_mutex, K_FOREVER);
    float smoothing_factor = SMOOTHING_FACTOR;
    k_mutex_unlock(&variable_mutex);

    static float smooth_x = 0, smooth_y = 0;
    smooth_x = smooth_value(smooth_x, delta_x, smoothing_factor);
    smooth_y = smooth_value(smooth_y, delta_y, smoothing_factor);

    // Calculate scaling based on movement frequency
    float scale = calculate_frequency_scale(movement_history);

    // Apply scaling and exponential function
    k_mutex_lock(&variable_mutex, K_FOREVER);
    float exponential_base = EXPONENTIAL_BASE;
    k_mutex_unlock(&variable_mutex);

    float scaled_x = apply_exponential_scaling(smooth_x * scale, exponential_base);
    float scaled_y = apply_exponential_scaling(smooth_y * scale, exponential_base);

    // Apply diagonal boost if movement is diagonal
    k_mutex_lock(&variable_mutex, K_FOREVER);
    if (is_diagonal) {
        scaled_x *= DIAGONAL_BOOST;
        scaled_y *= DIAGONAL_BOOST;
    }
    k_mutex_unlock(&variable_mutex);

    // Interpolate movement
    k_mutex_lock(&variable_mutex, K_FOREVER);
    uint8_t interpolation_steps = INTERPOLATION_STEPS;
    k_mutex_unlock(&variable_mutex);

    interpolate_movement(0, 0, scaled_x, scaled_y, interpolation_steps, dev);

    LOG_DBG("Processed movement: x=%.2f, y=%.2f, diagonal=%d", scaled_x, scaled_y, is_diagonal);
}

/* Forward declaration of functions */
static void pim447_work_handler(struct k_work *work);
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pim447_enable_interrupt(const struct pim447_config *config, bool enable);

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
    
    int8_t delta_x = (int8_t)buf[1] - (int8_t)buf[0];  // RIGHT - LEFT
    int8_t delta_y = (int8_t)buf[3] - (int8_t)buf[2];  // DOWN - UP

    // Only process non-zero movements
    if (delta_x != 0 || delta_y != 0) {
        // Update movement history
        movement_history[history_index].delta_x = delta_x;
        movement_history[history_index].delta_y = delta_y;
        movement_history[history_index].timestamp = k_uptime_get_32();

        history_index = (history_index + 1) % MOVEMENT_HISTORY_SIZE;

        // Process the movement
        process_movement(dev, (float)delta_x, (float)delta_y);
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
