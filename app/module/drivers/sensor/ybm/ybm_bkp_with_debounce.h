#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#define NUM_SAMPLES 1
#define SAMPLE_INTERVAL 50
#define MAX_READ_ATTEMPTS 20

struct ybm_config {
    const struct gpio_dt_spec a;
    const struct gpio_dt_spec b;

    const uint16_t steps;
    const uint8_t resolution;
};

struct ybm_data {
    int8_t ab_state;
    int8_t pulses;
    int8_t ticks;
    int8_t delta;
    // struct k_timer delay_timer;

    // Timer for re-read current state
    struct k_timer debounce_timer;
    uint8_t read_attempts;

    bool sampling_in_progress;

    // Debounce state
    uint8_t samples[NUM_SAMPLES]; // Array to store samples
    uint8_t current_sample_index; // Current position in samples array

#ifdef CONFIG_YBM_TRIGGER
    struct gpio_callback a_gpio_cb;
    struct gpio_callback b_gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

#if defined(CONFIG_YBM_TRIGGER_OWN_THREAD)
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_YBM_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif defined(CONFIG_YBM_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_YBM_TRIGGER */
};

#ifdef CONFIG_YBM_TRIGGER

int ybm_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                    sensor_trigger_handler_t handler);

int ybm_init_interrupt(const struct device *dev);
#endif