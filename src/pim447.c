#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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

/* Work handler function that logs when the trackball is moved */
static void pim447_work_handler(struct k_work *work) {
    struct pim447_data *data = CONTAINER_OF(work, struct pim447_data, work);

    /* Log that the trackball movement was detected */
    LOG_INF("Trackball moved");
}

/* GPIO callback function triggered by the interrupt */
static void pim447_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                 gpio_port_pins_t pins) {
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_gpio_cb);

    /* Schedule the work handler to process the movement */
    k_work_submit(&data->work);
}

/* Initialization function for the PIM447 driver */
static int pim447_init(const struct device *dev) {
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

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
    ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
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

    /* Initialize the work handler */
    k_work_init(&data->work, pim447_work_handler);

    /* Log that the driver has been initialized */
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
