/* pim447_behaviors.c */

/* SPDX-License-Identifier: MIT */

/* Define the driver compatibility */
#define DT_DRV_COMPAT zmk_behavior_pim447

#include <zephyr/kernel.h>
#include <drivers/behavior.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(zmk_behavior_pim447, CONFIG_ZMK_LOG_LEVEL);

#include "pimoroni_pim447.h"
#include "dt-bindings/behavior_pim447.h"

/* Extern variables */
extern volatile uint8_t PIM447_MOUSE_MAX_SPEED;
extern volatile uint8_t PIM447_MOUSE_MAX_TIME;
extern volatile float PIM447_MOUSE_SMOOTHING_FACTOR;
extern volatile uint8_t PIM447_SCROLL_MAX_SPEED;
extern volatile uint8_t PIM447_SCROLL_MAX_TIME;
extern volatile float PIM447_SCROLL_SMOOTHING_FACTOR;
extern volatile float PIM447_HUE_INCREMENT_FACTOR;

#define PIM447_MAX_SPEED_STEP 1
#define PIM447_MAX_TIME_STEP 1
#define PIM447_SMOOTHING_FACTOR_STEP 0.1f
#define PIM447_HUE_INCREMENT_FACTOR_STEP 0.1f

struct k_mutex variable_mutex;



/* Behavior function */
static int behavior_pim447_binding_pressed(struct zmk_behavior_binding *binding,
                                                     struct zmk_behavior_binding_event event)
{   const struct device *dev = device_get_binding(binding->behavior_dev); 
    if (!dev) {
        LOG_ERR("Device not found");
        return -ENODEV;
    }
    uint32_t action = binding->param1;  // Access the action parameter

    k_mutex_lock(&variable_mutex, K_FOREVER);

    switch (action) {
        case PIM447_MOUSE_INC_MAX_SPEED:
            PIM447_MOUSE_MAX_SPEED += PIM447_MAX_SPEED_STEP; // Increment max speed
            LOG_DBG("Mouse max speed increased to %d", PIM447_MOUSE_MAX_SPEED);
            break;
        case PIM447_MOUSE_DEC_MAX_SPEED:
            if (PIM447_MOUSE_MAX_SPEED > 1) {
                PIM447_MOUSE_MAX_SPEED -= PIM447_MAX_SPEED_STEP; // Decrement max speed
                LOG_DBG("Mouse max speed decreased to %d", PIM447_MOUSE_MAX_SPEED);
            }
            break;
        case PIM447_SCROLL_INC_MAX_SPEED:
            PIM447_SCROLL_MAX_SPEED += PIM447_MAX_SPEED_STEP; // Increment max speed
            LOG_DBG("Scroll max speed increased to %d", PIM447_SCROLL_MAX_SPEED);
            break;
        case PIM447_SCROLL_DEC_MAX_SPEED:
            if (PIM447_SCROLL_MAX_SPEED > 1) {
                PIM447_SCROLL_MAX_SPEED -= PIM447_MAX_SPEED_STEP; // Decrement max speed
                LOG_DBG("Scroll max speed decreased to %d", PIM447_SCROLL_MAX_SPEED);
            }
            break;
        case PIM447_MOUSE_INC_MAX_TIME:
            PIM447_MOUSE_MAX_TIME += PIM447_MAX_TIME_STEP; // Increment max TIME
            LOG_DBG("Mouse max time increased to %d", PIM447_MOUSE_MAX_TIME);
            break;
        case PIM447_MOUSE_DEC_MAX_TIME:
            if (PIM447_MOUSE_MAX_TIME > 5) {
                PIM447_MOUSE_MAX_TIME -= PIM447_MAX_TIME_STEP; // Decrement max TIME
                LOG_DBG("Mouse max time decreased to %d", PIM447_MOUSE_MAX_TIME);
            }
            break;
        case PIM447_SCROLL_INC_MAX_TIME:
            PIM447_MOUSE_MAX_TIME += PIM447_MAX_TIME_STEP; // Increment max TIME
            LOG_DBG("Scroll max time increased to %d", PIM447_MOUSE_MAX_TIME);
            break;
        case PIM447_SCROLL_DEC_MAX_TIME:
            if (PIM447_MOUSE_MAX_TIME > 5) {
                PIM447_MOUSE_MAX_TIME -= PIM447_MAX_TIME_STEP; // Decrement max TIME
                LOG_DBG("Scroll max time decreased to %d", PIM447_MOUSE_MAX_TIME);
            }
            break;
        case PIM447_MOUSE_INC_SMOOTHING_FACTOR:
            PIM447_MOUSE_SMOOTHING_FACTOR += PIM447_SMOOTHING_FACTOR_STEP; // Increment Smoothing factor
            LOG_DBG("Mouse Smoothing factor increased to %d.%02d", (int)PIM447_MOUSE_SMOOTHING_FACTOR, (int)(PIM447_MOUSE_SMOOTHING_FACTOR * 100) % 100);
            break;
        case PIM447_MOUSE_DEC_SMOOTHING_FACTOR:
            PIM447_MOUSE_SMOOTHING_FACTOR -= PIM447_SMOOTHING_FACTOR_STEP; // Decrement Smoothing factor
            LOG_DBG("Mouse Smoothing factor decreased to %d.%02d", (int)PIM447_MOUSE_SMOOTHING_FACTOR, (int)(PIM447_MOUSE_SMOOTHING_FACTOR * 100) % 100);
            break;
        case PIM447_SCROLL_INC_SMOOTHING_FACTOR:
            PIM447_SCROLL_SMOOTHING_FACTOR += PIM447_SMOOTHING_FACTOR_STEP; // Increment Smoothing factor
            LOG_DBG("Scroll Smoothing factor increased to %d.%02d", (int)PIM447_SCROLL_SMOOTHING_FACTOR, (int)(PIM447_SCROLL_SMOOTHING_FACTOR * 100) % 100);
            break;
        case PIM447_SCROLL_DEC_SMOOTHING_FACTOR:
            PIM447_SCROLL_SMOOTHING_FACTOR -= PIM447_SMOOTHING_FACTOR_STEP; // Decrement Smoothing factor
            LOG_DBG("Scroll Smoothing factor decreased to %d.%02d", (int)PIM447_SCROLL_SMOOTHING_FACTOR, (int)(PIM447_SCROLL_SMOOTHING_FACTOR * 100) % 100);
            break;
        case PIM447_INC_HUE_INCREMENT_FACTOR:
            PIM447_HUE_INCREMENT_FACTOR += PIM447_HUE_INCREMENT_FACTOR_STEP; // Increment Smoothing factor
            LOG_DBG("Hue increment factor increased to %d.%02d", (int)PIM447_HUE_INCREMENT_FACTOR, (int)(PIM447_HUE_INCREMENT_FACTOR * 100) % 100);
            break;
        case PIM447_DEC_HUE_INCREMENT_FACTOR:
            if (PIM447_HUE_INCREMENT_FACTOR > 0.1f) {
                PIM447_HUE_INCREMENT_FACTOR -= PIM447_HUE_INCREMENT_FACTOR_STEP; // Decrement Smoothing factor
                LOG_DBG("Hue increment factor decreased to %d.%02d", (int)PIM447_HUE_INCREMENT_FACTOR, (int)(PIM447_HUE_INCREMENT_FACTOR * 100) % 100);
            }
            break;
        case PIM447_TOGGLE_MODE:
            pim447_toggle_mode();
            break;
        case PIM447_ENABLE_SLEEP:
            pim447_enable_sleep(dev);
            break;
        case PIM447_DISABLE_SLEEP:
            pim447_disable_sleep(dev);
            break;
        default:
            LOG_WRN("Unknown trackball adjustment action: %d", action);
            break;
    }

    k_mutex_unlock(&variable_mutex);

    return -ENOTSUP;
}

/* Optionally implement the released function if needed */
static int behavior_pim447_binding_released(struct zmk_behavior_binding *binding,
                                                      struct zmk_behavior_binding_event event)
{
    // If your behavior needs to handle key releases, implement this function
    return ZMK_BEHAVIOR_OPAQUE;
}

/* Behavior driver API */
static const struct behavior_driver_api behavior_pim447_driver_api = {
    .binding_pressed = behavior_pim447_binding_pressed,
    .binding_released = behavior_pim447_binding_released,  // Set to NULL if not used
};

/* Initialization function (if needed) */
static int behavior_pim447_init(const struct device *dev)
{

    LOG_INF("Trackball adjustment behavior initialized");
    
    // Perform any initialization steps here
    return 0;
}

/* Register the behavior using BEHAVIOR_DT_INST_DEFINE */
BEHAVIOR_DT_INST_DEFINE(0,                                     // Instance number
                        behavior_pim447_init,        // Initialization function
                        NULL,                                  // PM control function
                        NULL,       // Data pointer
                        NULL,     // Configuration pointer
                        POST_KERNEL,                           // Initialization level
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   // Initialization priority
                        &behavior_pim447_driver_api  // Driver API pointer
);
