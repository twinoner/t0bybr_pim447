#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Extern variables */
extern volatile float speed_min;
extern struct k_mutex variable_mutex;

/* Constants for limits */
#define SPEED_MIN_MIN 0.1f
#define SPEED_MIN_MAX 10.0f
#define SPEED_STEP 0.1f

/* Increase function */
static int behavior_inc_speed_min_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                                         struct zmk_behavior_binding_event *event)
{

        LOG_INF("behavior_inc_speed_min_keymap_binding_pressed called");

    k_mutex_lock(&variable_mutex, K_FOREVER);

    speed_min += SPEED_STEP;
    if (speed_min > SPEED_MIN_MAX) {
        speed_min = SPEED_MIN_MAX;
    }

    LOG_INF("speed_min increased to %f", (double)speed_min);

    k_mutex_unlock(&variable_mutex);

    return ZMK_BEHAVIOR_OPAQUE;
}

/* Behavior driver API */
static const struct behavior_driver_api behavior_inc_speed_min_driver_api = {
    .binding_pressed = behavior_inc_speed_min_keymap_binding_pressed,
};

/* Register the behavior */
ZMK_BEHAVIOR_DEFINE(inc_speed_min, behavior_inc_speed_min_driver_api);
