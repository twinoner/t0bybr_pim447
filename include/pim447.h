#ifndef ZMK__DRIVERS__SENSORS__PIMORONI_PIM447_H
#define ZMK__DRIVERS__SENSORS__PIMORONI_PIM447_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

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

/* Control Masks */
#define MSK_CTRL_SLEEP      0b00000001
#define MSK_CTRL_RESET      0b00000010
#define MSK_CTRL_FREAD      0b00000100
#define MSK_CTRL_FWRITE     0b00001000

struct pimoroni_pim447_config {
    const struct device *i2c_bus;
    uint16_t i2c_addr;
#ifdef CONFIG_ZMK_SENSOR_PIMORONI_PIM447_INTERRUPT
    const struct gpio_dt_spec int_gpio;
#endif
};

struct pimoroni_pim447_data {
    const struct device *i2c_dev;
    const struct device *dev;  // Added to store the device pointer
#ifdef CONFIG_ZMK_SENSOR_PIMORONI_PIM447_INTERRUPT
    struct gpio_callback int_gpio_cb;
#endif
    struct k_work work;
    int8_t delta_left;
    int8_t delta_right;
    int8_t delta_up;
    int8_t delta_down;
    bool sw_pressed;
    bool sw_changed;
};

int pimoroni_pim447_led_set(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
int pimoroni_pim447_set_red(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_green(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_blue(const struct device *dev, uint8_t value);
int pimoroni_pim447_set_white(const struct device *dev, uint8_t value);

#endif /* ZMK__DRIVERS__SENSORS__PIMORONI_PIM447_H */
