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
#define DIRECTION_COUNT 4

#define BUTTON_DEBOUNCE_TIME_MS 50

#define LOG_FLOAT(value) ((int)(value * 100))  // Log float as integer * 100


/* Exposed variables */
volatile uint8_t FREQUENCY_THRESHOLD = 100;
volatile float BASE_SCALE_FACTOR = 1.0f;
volatile float MAX_SCALE_FACTOR = 5.0f;
volatile float SMOOTHING_FACTOR = 0.2f;
volatile uint8_t INTERPOLATION_STEPS = 5;
volatile float EXPONENTIAL_BASE = 1.5f;
volatile float DIAGONAL_THRESHOLD = 0.7f;
volatile float DIAGONAL_BOOST = 1.2f;
volatile uint8_t CALIBRATION_SAMPLES = 100;
volatile uint8_t  MOVEMENT_THRESHOLD = 1;

/* Mutex for thread safety */
K_MUTEX_DEFINE(variable_mutex);

LOG_MODULE_REGISTER(pim447, LOG_LEVEL_DBG);

enum direction {
    DIR_LEFT,
    DIR_RIGHT,
    DIR_UP,
    DIR_DOWN
};

struct direction_data {
    enum direction dir;
    int8_t value;
    uint32_t timestamp;
    const struct device *dev;
    struct k_work work;
};

/* Device configuration structure */
struct pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

/* Device data structure */
struct pim447_data {
    const struct device *dev;
    struct k_work_delayable work;
    struct gpio_callback int_gpio_cb;
    bool sw_pressed_prev;
    struct k_work direction_works[DIRECTION_COUNT];
    struct direction_data direction_data[DIRECTION_COUNT];
    float calibration_offsets[DIRECTION_COUNT];
    int calibration_count;
    struct k_work_q *trackball_workq;
    struct k_sem movement_sem;
    atomic_t accumulated_x;
    atomic_t accumulated_y;
};

struct movement_data {
    int8_t delta_x;
    int8_t delta_y;
    uint32_t timestamp;
};

K_THREAD_STACK_DEFINE(trackball_stack_area, 1024);
static struct k_work_q trackball_work_q;

static struct movement_data movement_history[MOVEMENT_HISTORY_SIZE];
static int history_index = 0;

/* Function prototypes */
static float smooth_value(float current, float target, float factor);
static float calculate_frequency_scale(const struct movement_data *history);
static float apply_exponential_scaling(float value, float base);
static void process_direction(struct k_work *work);
static void report_movement(struct k_work *work);
static void pim447_work_handler(struct k_work *work);
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int pim447_enable_interrupt(const struct pim447_config *config, bool enable);
static int pim447_enable(const struct device *dev);
static int pim447_disable(const struct device *dev);
static int pim447_init(const struct device *dev);

static float smooth_value(float current, float target, float factor) {
    return current;
}

static float apply_exponential_scaling(float value, float base) {
    return value;
}

static float apply_non_linear_scaling(float value) {
    return value;
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

static void process_direction(struct k_work *work)
{
    struct direction_data *data = CONTAINER_OF(work, struct direction_data, work);
    struct pim447_data *dev_data = CONTAINER_OF(data, struct pim447_data, direction_data[data->dir]);
    
    

    k_mutex_lock(&variable_mutex, K_FOREVER);
    float smoothing_factor = SMOOTHING_FACTOR;
    float exponential_base = EXPONENTIAL_BASE;
    k_mutex_unlock(&variable_mutex);

    // Apply calibration offset
    float calibrated_value = data->value - dev_data->calibration_offsets[data->dir];
    

    // Apply smoothing
    static float smooth_values[DIRECTION_COUNT] = {0};
    smooth_values[data->dir] = smooth_value(smooth_values[data->dir], calibrated_value, smoothing_factor);
    



    // Apply scaling and exponential function
    float scale = calculate_frequency_scale(movement_history);
    float scaled_value = apply_exponential_scaling(smooth_values[data->dir] * scale, exponential_base);
    
    
    // Apply non-linear scaling
    scaled_value = apply_non_linear_scaling(scaled_value);
    

     // Limit the final value to prevent overflow, but allow for finer movements
    float max_value = 1000.0f;
    scaled_value = fminf(fabsf(scaled_value), max_value) * (scaled_value >= 0 ? 1 : -1);

    // Accumulate movement
    switch (data->dir) {
        case DIR_LEFT:
            atomic_add(&dev_data->accumulated_x, (int32_t)(-scaled_value * 100));
            break;
        case DIR_RIGHT:
            atomic_add(&dev_data->accumulated_x, (int32_t)(scaled_value * 100));
            break;
        case DIR_UP:
            atomic_add(&dev_data->accumulated_y, (int32_t)(-scaled_value * 100));
            break;
        case DIR_DOWN:
            atomic_add(&dev_data->accumulated_y, (int32_t)(scaled_value * 100));
            break;
    }

    k_sem_give(&dev_data->movement_sem);

    
}

static void report_movement(struct k_work *work)
{
    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work.work);
    int32_t x_movement, y_movement;

    k_sem_take(&data->movement_sem, K_FOREVER);

    // Atomically get and reset accumulated movement
    x_movement = atomic_set(&data->accumulated_x, 0);
    y_movement = atomic_set(&data->accumulated_y, 0);

    

    // Scale down the movement to reasonable values
    int scaled_x = x_movement / 100;
    int scaled_y = y_movement / 100;

    // Report movement if it's non-zero
    if (scaled_x != 0 || scaled_y != 0) {
        int err;
        err = input_report_rel(data->dev, INPUT_REL_X, scaled_x, false, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report X movement: %d", err);
        } else {
            
        }
        err = input_report_rel(data->dev, INPUT_REL_Y, scaled_y, true, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to report Y movement: %d", err);
        } else {
            
        }
        
    } else {
        
    }

    // Schedule next report
    k_work_schedule(&data->work, K_MSEC(10));
}

static void pim447_work_handler(struct k_work *work)
{
    

    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work.work);
    const struct pim447_config *config = data->dev->config;
    const struct device *dev = data->dev;

    uint8_t buf[5];
    int ret;

    

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

    // Process each direction independently
    uint32_t current_time = k_uptime_get_32();
    for (int i = 0; i < DIRECTION_COUNT; i++) {
        data->direction_data[i].value = buf[i];
        data->direction_data[i].timestamp = current_time;
        data->direction_data[i].dev = dev;
        k_work_submit_to_queue(data->trackball_workq, &data->direction_works[i]);
    }

   static int64_t last_button_change = 0;
    int64_t now = k_uptime_get();

    bool sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    // Only report switch state change if debounce time has passed
    if (sw_pressed != data->sw_pressed_prev && 
        (now - last_button_change) > (int64_t)BUTTON_DEBOUNCE_TIME_MS) {
        int err = input_report_key(dev, INPUT_BTN_0, sw_pressed, true, K_FOREVER);
        if (err) {
            LOG_ERR("Failed to report switch state: %d", err);
        } else {
            LOG_DBG("Reported switch state: %d", sw_pressed);
        }
        data->sw_pressed_prev = sw_pressed;
        last_button_change = now;
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

    

    if (int_status & MSK_INT_TRIGGERED) {
        int_status &= ~MSK_INT_TRIGGERED;
        ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
        if (ret) {
            LOG_ERR("Failed to clear INT status register");
            return;
        }
        
    }
}

static void calibrate_trackball(struct pim447_data *data) {
    const struct pim447_config *config = data->dev->config;
    uint8_t buf[4];
    int ret;
    float sum[DIRECTION_COUNT] = {0};

    

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 4);
        if (ret) {
            LOG_ERR("Failed to read movement data during calibration");
            return;
        }

        for (int j = 0; j < DIRECTION_COUNT; j++) {
            sum[j] += buf[j];
        }

        k_msleep(10);  // Wait a bit between samples
    }

    for (int i = 0; i < DIRECTION_COUNT; i++) {
        data->calibration_offsets[i] = sum[i] / CALIBRATION_SAMPLES;
        
    }

    
}

static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_gpio_cb);
    const struct pim447_config *config = data->dev->config;

    

    int ret = k_work_submit(&data->work.work);
    if (ret < 0) {
        LOG_ERR("Failed to submit work item: %d", ret);
    } else {
        
    }
}

static int pim447_enable_interrupt(const struct pim447_config *config, bool enable) {
    uint8_t int_reg;
    int ret;

    /* Read the current INT register value */
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_reg);
    if (ret) {
        LOG_ERR("Failed to read INT register");
        return ret;
    }

    

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

    

    return 0;
}

static int pim447_enable(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    

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
    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_RAISING);
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

    

    return 0;
}

static int pim447_disable(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    

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

    

    return 0;
}

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
    

    /* Enable the Trackball */
    pim447_enable(dev);

    /* Initialize the work queue */
    k_work_queue_start(&trackball_work_q, trackball_stack_area,
                       K_THREAD_STACK_SIZEOF(trackball_stack_area),
                       CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);
    data->trackball_workq = &trackball_work_q;
    

    /* Initialize the work handler */
    k_work_init_delayable(&data->work, pim447_work_handler);
    

    /* Initialize direction-specific work items */
    for (int i = 0; i < DIRECTION_COUNT; i++) {
        k_work_init(&data->direction_works[i], process_direction);
        data->direction_data[i].dir = (enum direction)i;
    }

    /* Initialize movement semaphore */
    k_sem_init(&data->movement_sem, 0, 1);

    /* Perform initial calibration */
    calibrate_trackball(data);

    /* Schedule first movement report */
    k_work_schedule_for_queue(data->trackball_workq, &data->work, K_MSEC(10));
    

    

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