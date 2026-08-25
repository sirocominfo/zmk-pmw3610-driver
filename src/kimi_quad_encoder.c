/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT kimi_quad_encoder

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(KIMI_QUAD_ENCODER, CONFIG_SENSOR_LOG_LEVEL);

#define FULL_ROTATION 360

struct kqe_config {
    struct gpio_dt_spec a;
    struct gpio_dt_spec b;
    uint32_t steps;
    bool dummy;
};

struct kqe_data {
    const struct device *dev;
    struct gpio_callback a_cb;
    struct gpio_callback b_cb;

    /* デコード状態。割り込みコンテキストからのみ書き込む（irq_lockで保護）。
     * channel_get（スレッド文脈）もこの保護下で pulses を読み取る。 */
    uint8_t ab_state;
    int32_t pulses;

    const struct sensor_trigger *trigger;
    sensor_trigger_handler_t handler;

    struct k_sem sem;
    struct k_thread thread;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KIMI_QUAD_ENCODER_THREAD_STACK_SIZE);
};

static inline uint8_t kqe_ab_state(const struct kqe_config *cfg) {
    return (uint8_t)((gpio_pin_get_dt(&cfg->a) << 1) | gpio_pin_get_dt(&cfg->b));
}

/* app/module/drivers/sensor/ec11/ec11.c と全く同じデコード表。
 * ここだけはZMK本体のクアドラチャ解釈と互換を保つ（表自体は変更していない）。 */
static inline int8_t kqe_decode(uint8_t old_state, uint8_t new_state) {
    switch ((old_state << 2) | new_state) {
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        return -1;
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        return 1;
    default:
        return 0;
    }
}

/* 割り込みコンテキストから呼ばれる。ここが本ドライバの核心：
 * ピンを読んで演算するところまでを「割り込みを一切止めずに」その場でやり切る。
 * ZMK本体のec11ドライバは変化を検知した瞬間に両方の割り込みを止め、
 * 読み終わるまで再開しないため、その窓の間に来たもう片方の変化を
 * 丸ごと取りこぼす。ここではその窓自体を作らない
 * （gpio_pin_interrupt_configure_dt は kqe_init で一度呼ぶだけで、以後呼ばない）。 */
static void kqe_handle_edge(const struct device *dev) {
    const struct kqe_config *cfg = dev->config;
    struct kqe_data *data = dev->data;

    unsigned int key = irq_lock();
    uint8_t new_state = kqe_ab_state(cfg);
    int8_t delta = kqe_decode(data->ab_state, new_state);
    data->ab_state = new_state;
    data->pulses += delta;
    irq_unlock(key);

    if (delta != 0) {
        /* 時間のかかる「ZMKへ報告する」処理だけを専用スレッドへ逃がす。
         * GPIO割り込みはここでは一切触らない＝常に有効なまま。 */
        k_sem_give(&data->sem);
    }
}

static void kqe_a_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct kqe_data *data = CONTAINER_OF(cb, struct kqe_data, a_cb);
    kqe_handle_edge(data->dev);
}

static void kqe_b_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct kqe_data *data = CONTAINER_OF(cb, struct kqe_data, b_cb);
    kqe_handle_edge(data->dev);
}

static void kqe_thread_fn(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    struct kqe_data *data = p1;

    while (1) {
        k_sem_take(&data->sem, K_FOREVER);
        if (data->handler) {
            data->handler(data->dev, data->trigger);
        }
    }
}

static int kqe_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    ARG_UNUSED(dev);
    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION);
    /* デコードは割り込みコンテキストで完了済みなのでここでは何もしない。 */
    return 0;
}

static int kqe_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    const struct kqe_config *cfg = dev->config;
    struct kqe_data *data = dev->data;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    unsigned int key = irq_lock();
    int32_t pulses = data->pulses;
    data->pulses = 0;
    irq_unlock(key);

    /* alps,ec11 の steps>0 パスと完全に同じ換算式（behavior_sensor_rotate_common.c
     * が val1 を「角度(度)」として解釈するのに合わせる）。 */
    val->val1 = (pulses * FULL_ROTATION) / (int32_t)cfg->steps;
    val->val2 = (pulses * FULL_ROTATION) % (int32_t)cfg->steps;
    if (val->val2 != 0) {
        val->val2 *= 1000000;
        val->val2 /= (int32_t)cfg->steps;
    }

    return 0;
}

static int kqe_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler) {
    struct kqe_data *data = dev->data;

    data->trigger = trig;
    data->handler = handler;

    return 0;
}

static const struct sensor_driver_api kqe_driver_api = {
    .trigger_set = kqe_trigger_set,
    .sample_fetch = kqe_sample_fetch,
    .channel_get = kqe_channel_get,
};

static int kqe_init(const struct device *dev) {
    const struct kqe_config *cfg = dev->config;
    struct kqe_data *data = dev->data;

    data->dev = dev;

    if (!device_is_ready(cfg->a.port)) {
        LOG_ERR("A GPIO device is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->b.port)) {
        LOG_ERR("B GPIO device is not ready");
        return -ENODEV;
    }

    if (gpio_pin_configure_dt(&cfg->a, GPIO_INPUT)) {
        LOG_ERR("Failed to configure A pin");
        return -EIO;
    }
    if (gpio_pin_configure_dt(&cfg->b, GPIO_INPUT)) {
        LOG_ERR("Failed to configure B pin");
        return -EIO;
    }

    data->ab_state = kqe_ab_state(cfg);
    data->pulses = 0;

    k_sem_init(&data->sem, 0, K_SEM_MAX_LIMIT);

    /* dummyは実在のスイッチが無く報告する内容も無いので、そもそもこのスレッドを
     * 立てない（CPU・電池の無駄になるだけ）。 */
    if (!cfg->dummy) {
        k_thread_create(&data->thread, data->thread_stack,
                        K_KERNEL_STACK_SIZEOF(data->thread_stack), kqe_thread_fn, data, NULL,
                        NULL, K_PRIO_COOP(CONFIG_KIMI_QUAD_ENCODER_THREAD_PRIORITY), 0, K_NO_WAIT);
        k_thread_name_set(&data->thread, "kqe");
    }

    gpio_init_callback(&data->a_cb, kqe_a_callback, BIT(cfg->a.pin));
    if (gpio_add_callback(cfg->a.port, &data->a_cb) < 0) {
        LOG_ERR("Failed to set A callback");
        return -EIO;
    }
    gpio_init_callback(&data->b_cb, kqe_b_callback, BIT(cfg->b.pin));
    if (gpio_add_callback(cfg->b.port, &data->b_cb) < 0) {
        LOG_ERR("Failed to set B callback");
        return -EIO;
    }

    /* dummy=true の場合は割り込みを一切張らない（呼び出し元がGPIOを実在の
     * スイッチに繋がっていない旨を示した時に使う）。完全に浮いたピン(内部プルアップ
     * のみで支えている未接続ピン)は電気的ノイズを拾いやすく、割り込みを常時有効化
     * したままだとノイズだけで連続発火し得る（実機で、centralのダミーエンコーダー
     * 経由と思われるキー入力の乱れを確認、2026-08-26）。ダミー用途では回転を検出
     * する必要が無いので、そもそも割り込みを張らないのが最も安全。 */
    if (cfg->dummy) {
        return 0;
    }

    /* 起動時に一度だけ有効化し、以後は運用中一切無効化しない。
     * ec11ドライバが持つ「変化のたびに止めて処理し終えたら戻す」窓を
     * 最初から作らないことが、このドライバの核心。 */
    if (gpio_pin_interrupt_configure_dt(&cfg->a, GPIO_INT_EDGE_BOTH)) {
        LOG_ERR("Failed to configure A interrupt");
        return -EIO;
    }
    if (gpio_pin_interrupt_configure_dt(&cfg->b, GPIO_INT_EDGE_BOTH)) {
        LOG_ERR("Failed to configure B interrupt");
        return -EIO;
    }

    return 0;
}

#define KQE_INST(n)                                                                              \
    static struct kqe_data kqe_data_##n;                                                         \
    static const struct kqe_config kqe_cfg_##n = {                                               \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                   \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                   \
        .steps = DT_INST_PROP(n, steps),                                                          \
        .dummy = DT_INST_PROP(n, dummy),                                                          \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, kqe_init, NULL, &kqe_data_##n, &kqe_cfg_##n, POST_KERNEL,            \
                          CONFIG_SENSOR_INIT_PRIORITY, &kqe_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KQE_INST)
