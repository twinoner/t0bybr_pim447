/* behavior_trackball_adjust.c */

/* SPDX-License-Identifier: MIT */


#include <zephyr/kernel.h>
#include <drivers/behavior.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include "dt-bindings/trackball_actions.h"

/* Extern variables */
extern volatile uint8_t ACCUMULATION_THRESHOLD;
extern volatile float BASE_SCALE_FACTOR;
extern volatile float EXPONENTIAL_FACTOR;
extern volatile float SMOOTHING_FACTOR;
extern volatile uint8_t REPORT_INTERVAL_MS;
extern struct k_mutex variable_mutex;

/* Constants for limits and steps */
#define BASE_SCALE_STEP 0.1f
#define EXPONENTIAL_STEP 0.1f
#define SMOOTHING_STEP 0.05f

#define BASE_SCALE_MIN 0.5f
#define BASE_SCALE_MAX 20.0f
#define EXPONENTIAL_MIN 1.0f
#define EXPONENTIAL_MAX 5.0f
#define SMOOTHING_MIN 0.1f
#define SMOOTHING_MAX 2.0f

/* Define the driver compatibility */
#define DT_DRV_COMPAT zmk_behavior_trackball_adjust

/* Configuration and data structures */
struct behavior_trackball_adjust_config {
    // Add any configuration parameters if needed
};

struct behavior_trackball_adjust_data {
    // Add any runtime data if needed
};

/* Behavior function */
static int behavior_trackball_adjust_binding_pressed(struct zmk_behavior_binding *binding,
                                                     struct zmk_behavior_binding_event event)
{
    uint32_t action = binding->param1;  // Access the action parameter

    k_mutex_lock(&variable_mutex, K_FOREVER);

    switch (action) {
        case TB_INC_BASE_SCALE:
            BASE_SCALE_FACTOR += BASE_SCALE_STEP;
            if (BASE_SCALE_FACTOR > BASE_SCALE_MAX) {
                BASE_SCALE_FACTOR = BASE_SCALE_MAX;
            }
            LOG_INF("BASE_SCALE_FACTOR increased to %d.%02d", 
                    (int)BASE_SCALE_FACTOR, (int)(BASE_SCALE_FACTOR * 100) % 100);
            break;
        case TB_DEC_BASE_SCALE:
            BASE_SCALE_FACTOR -= BASE_SCALE_STEP;
            if (BASE_SCALE_FACTOR < BASE_SCALE_MIN) {
                BASE_SCALE_FACTOR = BASE_SCALE_MIN;
            }
            LOG_INF("BASE_SCALE_FACTOR decreased to %d.%02d", 
                    (int)BASE_SCALE_FACTOR, (int)(BASE_SCALE_FACTOR * 100) % 100);
            break;
        case TB_INC_EXPONENTIAL:
            EXPONENTIAL_FACTOR += EXPONENTIAL_STEP;
            if (EXPONENTIAL_FACTOR > EXPONENTIAL_MAX) {
                EXPONENTIAL_FACTOR = EXPONENTIAL_MAX;
            }
            LOG_INF("EXPONENTIAL_FACTOR increased to %d.%02d", 
                    (int)EXPONENTIAL_FACTOR, (int)(EXPONENTIAL_FACTOR * 100) % 100);
            break;
        case TB_DEC_EXPONENTIAL:
            EXPONENTIAL_FACTOR -= EXPONENTIAL_STEP;
            if (EXPONENTIAL_FACTOR < EXPONENTIAL_MIN) {
                EXPONENTIAL_FACTOR = EXPONENTIAL_MIN;
            }
            LOG_INF("EXPONENTIAL_FACTOR decreased to %d.%02d", 
                    (int)EXPONENTIAL_FACTOR, (int)(EXPONENTIAL_FACTOR * 100) % 100);
            break;
        case TB_INC_SMOOTHING:
            SMOOTHING_FACTOR += SMOOTHING_STEP;
            if (SMOOTHING_FACTOR > SMOOTHING_MAX) {
                SMOOTHING_FACTOR = SMOOTHING_MAX;
            }
            LOG_INF("SMOOTHING_FACTOR increased to %d.%02d", 
                    (int)SMOOTHING_FACTOR, (int)(SMOOTHING_FACTOR * 100) % 100);
            break;
        case TB_DEC_SMOOTHING:
            SMOOTHING_FACTOR -= SMOOTHING_STEP;
            if (SMOOTHING_FACTOR < SMOOTHING_MIN) {
                SMOOTHING_FACTOR = SMOOTHING_MIN;
            }
            LOG_INF("SMOOTHING_FACTOR decreased to %d.%02d", 
                    (int)SMOOTHING_FACTOR, (int)(SMOOTHING_FACTOR * 100) % 100);
            break;
        default:
            LOG_WRN("Unknown trackball adjustment action: %d", action);
            break;
    }

    k_mutex_unlock(&variable_mutex);

    return 0;
}

/* Optionally implement the released function if needed */
static int behavior_trackball_adjust_binding_released(struct zmk_behavior_binding *binding,
                                                      struct zmk_behavior_binding_event event)
{
    // If your behavior needs to handle key releases, implement this function
    return 0;
}

/* Behavior driver API */
static const struct behavior_driver_api behavior_trackball_adjust_driver_api = {
    .binding_pressed = behavior_trackball_adjust_binding_pressed,
    .binding_released = NULL,  // Set to NULL if not used
};

/* Initialization function (if needed) */
static int behavior_trackball_adjust_init(const struct device *dev)
{

    LOG_INF("Trackball adjustment behavior initialized");
    
    // Perform any initialization steps here
    return 0;
}

/* Device instance definition */
static struct behavior_trackball_adjust_data behavior_trackball_adjust_data;

static const struct behavior_trackball_adjust_config behavior_trackball_adjust_config = {
    // Initialize configuration if needed
};

/* Register the behavior using BEHAVIOR_DT_INST_DEFINE */
BEHAVIOR_DT_INST_DEFINE(0,                                     // Instance number
                        behavior_trackball_adjust_init,        // Initialization function
                        NULL,                                  // PM control function
                        &behavior_trackball_adjust_data,       // Data pointer
                        &behavior_trackball_adjust_config,     // Configuration pointer
                        POST_KERNEL,                           // Initialization level
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   // Initialization priority
                        &behavior_trackball_adjust_driver_api  // Driver API pointer
);
