#include "periph_encoder.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "freertos/queue.h"
#include "audio_mem.h"

#define TAG "periph_encoder"

#define PERIPH_ID_ENCODER 20

typedef struct {
    pcnt_unit_handle_t pcnt_unit;
    QueueHandle_t      queue;
    int                step;
} encoder_ctx_t;

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                     const pcnt_watch_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t wakeup;
    QueueHandle_t q = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(q, &edata->watch_point_value, &wakeup);
    return wakeup == pdTRUE;
}

static esp_err_t encoder_init(esp_periph_handle_t self)
{
    // nothing extra needed, PCNT already started in periph_encoder_init
    return ESP_OK;
}

static esp_err_t encoder_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
    return ESP_OK;
}

static esp_err_t encoder_destroy(esp_periph_handle_t self)
{
    encoder_ctx_t *ctx = esp_periph_get_data(self);
    pcnt_unit_stop(ctx->pcnt_unit);
    pcnt_unit_disable(ctx->pcnt_unit);
    pcnt_del_unit(ctx->pcnt_unit);
    vQueueDelete(ctx->queue);
    free(ctx);
    return ESP_OK;
}

// at file scope, store the periph handle for the timer callback to access
static esp_periph_handle_t s_encoder_periph = NULL;

static void encoder_timer_cb(xTimerHandle tmr)
{
    if (s_encoder_periph == NULL) return;
    encoder_ctx_t *ctx = esp_periph_get_data(s_encoder_periph);
    int event_val;

    if (xQueueReceive(ctx->queue, &event_val, 0)) {
        pcnt_unit_clear_count(ctx->pcnt_unit);
        if (event_val > 0) {
            esp_periph_send_event(s_encoder_periph, PERIPH_ENCODER_CW, NULL, 0);
        } else {
            esp_periph_send_event(s_encoder_periph, PERIPH_ENCODER_CCW, NULL, 0);
        }
    }
}

esp_periph_handle_t periph_encoder_init(periph_encoder_cfg_t *cfg)
{
    encoder_ctx_t *ctx = audio_calloc(1, sizeof(encoder_ctx_t));
    ctx->step  = cfg->step > 0 ? cfg->step : 4;
    ctx->queue = xQueueCreate(10, sizeof(int));

    pcnt_unit_config_t unit_cfg = {
        .high_limit =  ctx->step,
        .low_limit  = -ctx->step,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &ctx->pcnt_unit));

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ctx->pcnt_unit, &filter));

    pcnt_channel_handle_t ch_a, ch_b;
    pcnt_chan_config_t chan_a = {
        .edge_gpio_num  = cfg->gpio_a,
        .level_gpio_num = cfg->gpio_b,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->pcnt_unit, &chan_a, &ch_a));

    pcnt_chan_config_t chan_b = {
        .edge_gpio_num  = cfg->gpio_b,
        .level_gpio_num = cfg->gpio_a,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->pcnt_unit, &chan_b, &ch_b));

    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,   PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ctx->pcnt_unit,  ctx->step));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ctx->pcnt_unit, -ctx->step));

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_on_reach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(ctx->pcnt_unit, &cbs, ctx->queue));

    ESP_ERROR_CHECK(pcnt_unit_enable(ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(ctx->pcnt_unit));

    esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_ENCODER, "encoder");
    s_encoder_periph = periph;  // store for timer callback
    esp_periph_set_data(periph, ctx);
    esp_periph_set_function(periph, encoder_init, encoder_run, encoder_destroy);
    esp_periph_start_timer(periph, pdMS_TO_TICKS(10), encoder_timer_cb);

    return periph;
}