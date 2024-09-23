/* behavior_inc_speed_max.c */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zmk/behavior.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Extern variables */
extern volatile float speed_max;
extern struct k_mutex variable_mutex;

/* Constants for limits */
#define SPEED_MAX_MIN 0.1f
#define SPEED_MAX_MAX 20.0f
#define SPEED_STEP 0.1f

/* Increase function */
static int on_inc_speed_max(struct zmk_behavior_binding *binding,
                            struct zmk_behavior_binding_event event)
{
    if (event.position != ZMK_POSITION_STATE_RELEASED) {
        k_mutex_lock(&variable_mutex, K_FOREVER);

        speed_max += SPEED_STEP;
        if (speed_max > SPEED_MAX_MAX) {
            speed_max = SPEED_MAX_MAX;
        }

        LOG_INF("speed_max increased to %f", (double)speed_max);

        k_mutex_unlock(&variable_mutex);
    }
    return 0;
}

/* Behavior driver API */
static const struct behavior_driver_api behavior_inc_api = {
    .binding_pressed = on_inc_speed_max,
};

/* Device registration */
DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                      APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      &behavior_inc_api);
