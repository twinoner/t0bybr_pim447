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
extern volatile uint8_t FREQUENCY_THRESHOLD;
extern volatile float BASE_SCALE_FACTOR;
extern volatile float MAX_SCALE_FACTOR;
extern volatile float SMOOTHING_FACTOR;
extern volatile uint8_t INTERPOLATION_STEPS;
extern volatile float EXPONENTIAL_BASE;
extern struct k_mutex variable_mutex;

/* Constants for limits and steps */
#define BASE_SCALE_STEP 0.1f
#define FREQUENCY_THRESHOLD_STEP 5
#define INTERPOLATION_STEPS_STEP 1
#define EXPONENTIAL_BASE_STEP 0.1f
#define SMOOTHING_FACTOR_STEP 0.05f

#define BASE_SCALE_MIN 0.1f
#define BASE_SCALE_MAX 5.0f
#define FREQUENCY_THRESHOLD_MIN 0
#define FREQUENCY_THRESHOLD_MAX 255
#define SMOOTHING_FACTOR_MIN 0.0f
#define SMOOTHING_FACTOR_MAX 1.0f
#define INTERPOLATION_STEPS_MIN 1
#define INTERPOLATION_STEPS_MAX 10
#define EXPONENTIAL_BASE_MIN 1.1f
#define EXPONENTIAL_BASE_MAX 2.0f

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
        case TB_INC_FREQUENCY_THRESHOLD:
            FREQUENCY_THRESHOLD += FREQUENCY_THRESHOLD_STEP;
            if (FREQUENCY_THRESHOLD > FREQUENCY_THRESHOLD_MAX) {
                FREQUENCY_THRESHOLD = FREQUENCY_THRESHOLD_MAX;
            }
            LOG_INF("FREQUENCY_THRESHOLD increased to %d", (int)FREQUENCY_THRESHOLD);
            break;
        case TB_DEC_FREQUENCY_THRESHOLD:
            FREQUENCY_THRESHOLD -= FREQUENCY_THRESHOLD_STEP;
            if (FREQUENCY_THRESHOLD < FREQUENCY_THRESHOLD_MIN) {
                FREQUENCY_THRESHOLD = FREQUENCY_THRESHOLD_MIN;
            }
            LOG_INF("FREQUENCY_THRESHOLD decreased to %d", (int)FREQUENCY_THRESHOLD);
            break;
        case TB_INC_SMOOTHING_FACTOR:
            SMOOTHING_FACTOR += SMOOTHING_FACTOR_STEP;
            if (SMOOTHING_FACTOR > SMOOTHING_FACTOR_MAX) {
                SMOOTHING_FACTOR = SMOOTHING_FACTOR_MAX;
            }
            LOG_INF("SMOOTHING_FACTOR increased to %.2f", SMOOTHING_FACTOR);
            break;
        case TB_DEC_SMOOTHING_FACTOR:
            SMOOTHING_FACTOR -= SMOOTHING_FACTOR_STEP;
            if (SMOOTHING_FACTOR < SMOOTHING_FACTOR_MIN) {
                SMOOTHING_FACTOR = SMOOTHING_FACTOR_MIN;
            }
            LOG_INF("SMOOTHING_FACTOR decreased to %.2f", SMOOTHING_FACTOR);
            break;
        case TB_INC_INTERPOLATION_STEPS:
            INTERPOLATION_STEPS += INTERPOLATION_STEPS_STEP;
            if (INTERPOLATION_STEPS > INTERPOLATION_STEPS_MAX) {
                INTERPOLATION_STEPS = INTERPOLATION_STEPS_MAX;
            }
            LOG_INF("INTERPOLATION_STEPS increased to %d", INTERPOLATION_STEPS);
            break;
        case TB_DEC_INTERPOLATION_STEPS:
            INTERPOLATION_STEPS -= INTERPOLATION_STEPS_STEP;
            if (INTERPOLATION_STEPS < INTERPOLATION_STEPS_MIN) {
                INTERPOLATION_STEPS = INTERPOLATION_STEPS_MIN;
            }
            LOG_INF("INTERPOLATION_STEPS decreased to %d", INTERPOLATION_STEPS);
            break;
        case TB_INC_EXPONENTIAL_BASE:
            EXPONENTIAL_BASE += EXPONENTIAL_BASE_STEP;
            if (EXPONENTIAL_BASE > EXPONENTIAL_BASE_MAX) {
                EXPONENTIAL_BASE = EXPONENTIAL_BASE_MAX;
            }
            LOG_INF("EXPONENTIAL_BASE increased to %.2f", EXPONENTIAL_BASE);
            break;
        case TB_DEC_EXPONENTIAL_BASE:
            EXPONENTIAL_BASE -= EXPONENTIAL_BASE_STEP;
            if (EXPONENTIAL_BASE < EXPONENTIAL_BASE_MIN) {
                EXPONENTIAL_BASE = EXPONENTIAL_BASE_MIN;
            }
            LOG_INF("EXPONENTIAL_BASE decreased to %.2f", EXPONENTIAL_BASE);
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
