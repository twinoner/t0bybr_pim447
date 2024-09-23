/* behavior_trackball_adjust.c */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <dt-bindings/trackball_actions.h>

/* Extern variables */
extern volatile float speed_min;
extern volatile float speed_max;
extern volatile float scale_divisor_min;
extern volatile float scale_divisor_max;
extern struct k_mutex variable_mutex;

/* Constants for limits and steps */
#define SPEED_MIN_MIN 0.1f
#define SPEED_MIN_MAX 10.0f
#define SPEED_MAX_MIN 0.1f
#define SPEED_MAX_MAX 20.0f
#define SCALE_DIV_MIN_MIN 0.1f
#define SCALE_DIV_MIN_MAX 5.0f
#define SCALE_DIV_MAX_MIN 0.1f
#define SCALE_DIV_MAX_MAX 5.0f

#define SPEED_STEP 0.1f
#define SCALE_DIV_STEP 0.1f

static int behavior_trackball_adjust_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                                            struct zmk_behavior_binding_event *event)
{
    uint32_t action = binding->parameters.u32[0];  // Get the action parameter

    k_mutex_lock(&variable_mutex, K_FOREVER);

    switch (action) {
    case TB_INC_SPEED_MIN:
        speed_min += SPEED_STEP;
        if (speed_min > SPEED_MIN_MAX) {
            speed_min = SPEED_MIN_MAX;
        }
        LOG_INF("speed_min increased to %f", (double)speed_min);
        break;
    case TB_DEC_SPEED_MIN:
        speed_min -= SPEED_STEP;
        if (speed_min < SPEED_MIN_MIN) {
            speed_min = SPEED_MIN_MIN;
        }
        LOG_INF("speed_min decreased to %f", (double)speed_min);
        break;
    case TB_INC_SPEED_MAX:
        speed_max += SPEED_STEP;
        if (speed_max > SPEED_MAX_MAX) {
            speed_max = SPEED_MAX_MAX;
        }
        LOG_INF("speed_max increased to %f", (double)speed_max);
        break;
    case TB_DEC_SPEED_MAX:
        speed_max -= SPEED_STEP;
        if (speed_max < SPEED_MAX_MIN) {
            speed_max = SPEED_MAX_MIN;
        }
        LOG_INF("speed_max decreased to %f", (double)speed_max);
        break;
    case TB_INC_SCALE_DIV_MIN:
        scale_divisor_min += SCALE_DIV_STEP;
        if (scale_divisor_min > SCALE_DIV_MIN_MAX) {
            scale_divisor_min = SCALE_DIV_MIN_MAX;
        }
        LOG_INF("scale_divisor_min increased to %f", (double)scale_divisor_min);
        break;
    case TB_DEC_SCALE_DIV_MIN:
        scale_divisor_min -= SCALE_DIV_STEP;
        if (scale_divisor_min < SCALE_DIV_MIN_MIN) {
            scale_divisor_min = SCALE_DIV_MIN_MIN;
        }
        LOG_INF("scale_divisor_min decreased to %f", (double)scale_divisor_min);
        break;
    case TB_INC_SCALE_DIV_MAX:
        scale_divisor_max += SCALE_DIV_STEP;
        if (scale_divisor_max > SCALE_DIV_MAX_MAX) {
            scale_divisor_max = SCALE_DIV_MAX_MAX;
        }
        LOG_INF("scale_divisor_max increased to %f", (double)scale_divisor_max);
        break;
    case TB_DEC_SCALE_DIV_MAX:
        scale_divisor_max -= SCALE_DIV_STEP;
        if (scale_divisor_max < SCALE_DIV_MAX_MIN) {
            scale_divisor_max = SCALE_DIV_MAX_MIN;
        }
        LOG_INF("scale_divisor_max decreased to %f", (double)scale_divisor_max);
        break;
    default:
        LOG_WRN("Unknown trackball adjustment action: %d", action);
        break;
    }

    k_mutex_unlock(&variable_mutex);

    return ZMK_BEHAVIOR_OPAQUE;
}

/* Behavior driver API */
static const struct behavior_driver_api behavior_trackball_adjust_driver_api = {
    .binding_pressed = behavior_trackball_adjust_keymap_binding_pressed,
};

/* Register the behavior */
ZMK_BEHAVIOR_DEFINE(trackball_adjust, behavior_trackball_adjust_driver_api);
