#ifndef PIMORONI_PIM447_H
#define PIMORONI_PIM447_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mutex.h>

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

#define LED_ANIMATION_INTERVAL_MS 50 // Update interval in milliseconds

typedef enum {
    PIM447_LED_RED,
    PIM447_LED_GREEN,
    PIM447_LED_BLUE,
    PIM447_LED_WHITE
} pim447_led_t;

struct pimoroni_pim447_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct pimoroni_pim447_data {
    const struct device *dev;
    struct gpio_callback int_gpio_cb;
    struct k_work_delayable periodic_work;
    int16_t delta_x;
    int16_t delta_y;
    struct k_mutex data_lock;
    bool sw_pressed;
    bool sw_pressed_prev;
    struct k_work_delayable led_work;
    float hue;
    bool led_animation_running;

};

int pimoroni_pim447_set_leds(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
int pimoroni_pim447_set_led(const struct device *dev, pim447_led_t led, uint8_t brightness);

void pimoroni_pim447_start_led_animation(const struct device *dev);
void pimoroni_pim447_stop_led_animation(const struct device *dev);

#endif /* PIMORONI_PIM447_H */
