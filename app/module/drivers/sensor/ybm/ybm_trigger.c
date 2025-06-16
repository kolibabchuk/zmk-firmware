
#define DT_DRV_COMPAT alps_ybm

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "ybm.h"

extern struct ybm_data ybm_driver;

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(YBM, CONFIG_SENSOR_LOG_LEVEL);

/**
 * Enable/Disable interrupts for pins
 */
static inline void setup_int(const struct device *dev, bool enable) {
    const struct ybm_config *cfg = dev->config;

    if (gpio_pin_interrupt_configure_dt(&cfg->a, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
    }

    if (gpio_pin_interrupt_configure_dt(&cfg->b, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
    }
}

/**
 * Common callback function for both A and B pins
 */
static void ybm_gpio_callback_common(struct ybm_data *drv_data) {
    if (drv_data->sampling_in_progress) {
        return;
    }
    drv_data->sampling_in_progress = true;

    setup_int(drv_data->dev, false);

#if defined(CONFIG_YBM_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_YBM_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

/**
 * Triggered from interrupt A
 */
static void ybm_a_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ybm_gpio_callback_common(CONTAINER_OF(cb, struct ybm_data, a_gpio_cb));
}

/**
 * Triggered from interrupt B
 */
static void ybm_b_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ybm_gpio_callback_common(CONTAINER_OF(cb, struct ybm_data, b_gpio_cb));
}

/**
 * Will be triggered during the interrupt processing.
 */
static void ybm_thread_cb(const struct device *dev) {
    struct ybm_data *drv_data = dev->data;

    drv_data->handler(dev, drv_data->trigger);

    setup_int(dev, true);
}

#ifdef CONFIG_YBM_TRIGGER_OWN_THREAD
static void ybm_thread(int dev_ptr, int unused) {
    const struct device *dev = INT_TO_POINTER(dev_ptr);
    struct ybm_data *drv_data = dev->data;

    ARG_UNUSED(unused);

    while (1) {
        k_sem_take(&drv_data->gpio_sem, K_FOREVER);
        ybm_thread_cb(dev);
    }
}
#endif

/**
 * Callback to be triggered by the interrupts.
 * When k_work_submit(&drv_data->work) is called, this callback will be triggered.
 */
#ifdef CONFIG_YBM_TRIGGER_GLOBAL_THREAD
static void ybm_work_cb(struct k_work *work) {
    struct ybm_data *drv_data = CONTAINER_OF(work, struct ybm_data, work);
    ybm_thread_cb(drv_data->dev);
}
#endif

int ybm_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                    sensor_trigger_handler_t handler) {
    struct ybm_data *drv_data = dev->data;

    setup_int(dev, false);

    k_msleep(5);

    drv_data->trigger = trig;
    drv_data->handler = handler;

    setup_int(dev, true);

    return 0;
}

/**
 * Called from ybm_init to initialize interrupts for sensor rotate
 */
int ybm_init_interrupt(const struct device *dev) {
    struct ybm_data *drv_data = dev->data;
    const struct ybm_config *drv_cfg = dev->config;

    drv_data->dev = dev;

    // setup gpio interrupt for A pin
    gpio_init_callback(&drv_data->a_gpio_cb, ybm_a_gpio_callback, BIT(drv_cfg->a.pin));
    if (gpio_add_callback(drv_cfg->a.port, &drv_data->a_gpio_cb) < 0) {
        LOG_DBG("Failed to set A callback!");
        return -EIO;
    }

    // // setup gpio interrupt for B pin
    gpio_init_callback(&drv_data->b_gpio_cb, ybm_b_gpio_callback, BIT(drv_cfg->b.pin));
    if (gpio_add_callback(drv_cfg->b.port, &drv_data->b_gpio_cb) < 0) {
        LOG_DBG("Failed to set B callback!");
        return -EIO;
    }

#if defined(CONFIG_YBM_TRIGGER_OWN_THREAD)
    k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

    k_thread_create(&drv_data->thread, drv_data->thread_stack, CONFIG_YBM_THREAD_STACK_SIZE,
                    (k_thread_entry_t)ybm_thread, dev, 0, NULL,
                    K_PRIO_COOP(CONFIG_YBM_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_YBM_TRIGGER_GLOBAL_THREAD)
    k_work_init(&drv_data->work, ybm_work_cb);
#endif

    return 0;
}