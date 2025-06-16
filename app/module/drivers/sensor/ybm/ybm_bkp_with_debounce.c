/*
Application
    ↓
sensor_sample_fetch()
    ↓
ybm_sample_fetch()  // Starts sampling with debouncing
    ↓
[debouncing happens]
    ↓
Application
    ↓
sensor_channel_get()
    ↓
ybm_channel_get()   // Gets the processed data

*/

#define DT_DRV_COMPAT ysa_ybm

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "ybm.h"

#define FULL_ROTATION 360

LOG_MODULE_REGISTER(YBM, CONFIG_SENSOR_LOG_LEVEL);

static int8_t ybm_get_ab_state(const struct device *dev) {
    const struct ybm_config *drv_cfg = dev->config;

    return (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
}

static int ybm_get_rotation_direction(uint8_t prev, uint8_t curr) {
    // static const int8_t directionMap[16] = {
    //     0,  0, 0, 1, // 00→00, 00→01, 00→10, 00→11
    //     0,  0, 0, 0, // 01→00, 01→01, 01→10, 01→11
    //     0,  0, 0, 0, // 10→00, 10→01, 10→10, 10→11
    //     -1, 0, 0, 0  // 11→00, 11→01, 11→10, 11→11
    // };

    // return directionMap[(prev << 2) | curr];

    if (prev == curr) {
        return 0;
    }

    if ((prev == 0 && curr == 3) || (curr == 0 && (prev == 1 || prev == 2))) {
        return 1;
    } else if ((prev == 3 && curr == 0) || (curr == 3 && (prev == 1 || prev == 2))) {
        return -1;
    }

    return 0;
}

static void debounce_timer_handler(struct k_timer *timer) {
    struct ybm_data *drv_data = CONTAINER_OF(timer, struct ybm_data, debounce_timer);
    const struct device *dev = drv_data->dev;
    int8_t current_sample;

    // Get current sample
    current_sample = ybm_get_ab_state(dev);

    // Store the sample
    drv_data->samples[drv_data->current_sample_index] = current_sample;
    drv_data->current_sample_index++;

    // If we haven't collected all samples yet, schedule next sample
    if (NUM_SAMPLES > 1 && drv_data->current_sample_index < NUM_SAMPLES) {
        LOG_WRN("Sample: %d, value: %d", drv_data->current_sample_index, current_sample);
        k_timer_start(&drv_data->debounce_timer, K_USEC(SAMPLE_INTERVAL), K_NO_WAIT);
        return;
    }

    int8_t most_frequent_state = 0;
    if (NUM_SAMPLES > 1) {
        // Count occurrences of each state
        uint8_t state_counts[4] = {0}; // Count for each possible state (00, 01, 10, 11)
        for (int i = 0; i < NUM_SAMPLES; i++) {
            state_counts[drv_data->samples[i]]++;
        }

        // Find the most frequent state
        uint8_t max_count = 0;
        for (int i = 0; i < 4; i++) {
            if (state_counts[i] > max_count) {
                max_count = state_counts[i];
                most_frequent_state = i;
            }
        }
    } else {
        most_frequent_state = current_sample;
    }

    if (drv_data->ab_state == most_frequent_state) {
        while (drv_data->read_attempts < MAX_READ_ATTEMPTS &&
               drv_data->ab_state == most_frequent_state) {
            LOG_WRN("Meet the same value: %d, attemmpt: %d", most_frequent_state,
                    drv_data->read_attempts);
            drv_data->read_attempts++;
            most_frequent_state = ybm_get_ab_state(dev);
        }
        // drv_data->read_attempts++;
        // if (MAX_READ_ATTEMPTS > 1 && drv_data->read_attempts < MAX_READ_ATTEMPTS) {
        //     LOG_WRN("Meet the same value: %d, attemmpt: %d", most_frequent_state,
        //             drv_data->read_attempts);
        //             k_sleep(K_MSEC(1));
        //     k_timer_start(&drv_data->debounce_timer, K_USEC(SAMPLE_INTERVAL), K_NO_WAIT);
        //     return;
        // }
    }

    int8_t delta = ybm_get_rotation_direction(drv_data->ab_state, most_frequent_state);

    LOG_WRN("prev: %d, new: %d, delta: %d", drv_data->ab_state, most_frequent_state, delta);

    if (delta != 0) { // Only process if there's actual movement
        drv_data->pulses += delta;
    }
    drv_data->ab_state = most_frequent_state;

    // Reset sampling state
    drv_data->sampling_in_progress = false;
    drv_data->current_sample_index = 0;
    drv_data->read_attempts = 0;
}

static int ybm_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct ybm_data *drv_data = dev->data;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION);

    // Reset sampling state
    drv_data->current_sample_index = 0;
    drv_data->sampling_in_progress = true;
    drv_data->read_attempts = 0;

    // Start timer for first sample
    k_timer_start(&drv_data->debounce_timer, K_USEC(SAMPLE_INTERVAL), K_NO_WAIT);

    return 0;
}

static int ybm_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    struct ybm_data *drv_data = dev->data;
    const struct ybm_config *drv_cfg = dev->config;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    // Wait for sampling to complete if it's in progress
    if (drv_data->sampling_in_progress) {
        //  Wait for the debouncing to complete
        uint8_t tl = 100;
        while (drv_data->sampling_in_progress && tl) {
            LOG_WRN("sampling_in_progress");
            tl--;
            k_sleep(K_MSEC(1)); // Small sleep to prevent busy waiting
        }
    }

    int32_t pulses = drv_data->pulses;
    drv_data->pulses = 0;

    val->val1 = (pulses * FULL_ROTATION) / drv_cfg->steps;
    val->val2 = (pulses * FULL_ROTATION) % drv_cfg->steps;
    if (val->val2 != 0) {
        val->val2 *= 1000000;
        val->val2 /= drv_cfg->steps;
    }

    return 0;
}

static const struct sensor_driver_api ybm_driver_api = {
#ifdef CONFIG_YBM_TRIGGER
    .trigger_set = ybm_trigger_set,
#endif
    .sample_fetch = ybm_sample_fetch,
    .channel_get = ybm_channel_get,
};

int ybm_init(const struct device *dev) {
    struct ybm_data *drv_data = dev->data;
    const struct ybm_config *drv_cfg = dev->config;

    // Store the device pointer
    drv_data->dev = dev;

    if (!device_is_ready(drv_cfg->a.port)) {
        LOG_ERR("A GPIO device is not ready");
        return -EINVAL;
    }

    if (!device_is_ready(drv_cfg->b.port)) {
        LOG_ERR("B GPIO device is not ready");
        return -EINVAL;
    }

    if (gpio_pin_configure_dt(&drv_cfg->a, GPIO_INPUT)) {
        LOG_DBG("Failed to configure A pin");
        return -EIO;
    }

    if (gpio_pin_configure_dt(&drv_cfg->b, GPIO_INPUT)) {
        LOG_DBG("Failed to configure B pin");
        return -EIO;
    }

#ifdef CONFIG_YBM_TRIGGER
    if (ybm_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }
#endif

    // Initialize the debounce timer
    k_timer_init(&drv_data->debounce_timer, debounce_timer_handler, NULL);

    drv_data->sampling_in_progress = false;
    drv_data->current_sample_index = 0;
    drv_data->read_attempts = 0;

    drv_data->ab_state = ybm_get_ab_state(dev);

    return 0;
}

#define YBM_INST(n)                                                                                \
    static struct ybm_data ybm_data_##n;                                                           \
    static const struct ybm_config ybm_cfg_##n = {                                                 \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .resolution = DT_INST_PROP_OR(n, resolution, 1),                                           \
        .steps = DT_INST_PROP_OR(n, steps, 0),                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ybm_init, NULL, &ybm_data_##n, &ybm_cfg_##n, POST_KERNEL,             \
                          CONFIG_SENSOR_INIT_PRIORITY, &ybm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(YBM_INST)