#ifndef PIMORONI_PIM447_H
#define PIMORONI_PIM447_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

#define I2C1_NODE DT_NODELABEL(pimoroni_pim447)



static const struct i2c_dt_spec pimoroni_pim447_i2c = I2C_DT_SPEC_GET(I2C1_NODE); 


struct pimoroni_pim447_config {
    const struct device *i2c_bus;

    uint16_t i2c_addr;
    const struct gpio_dt_spec int_gpio;
};

struct pimoroni_pim447_data {
    const struct device *i2c_dev;
    const struct device *dev;
    struct gpio_callback int_gpio_cb;
    struct k_work work;
    bool sw_pressed;
};

int pimoroni_pim447_led_set(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
int pimoroni_pim447_set_red(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_green(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_blue(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_white(const struct device *dev, uint8_t value);

#endif /* PIMORONI_PIM447_H */
