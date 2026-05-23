#include <math.h>
#include <string.h>
#include "sine_generator.h"
#include "audio_mem.h"
#include "esp_log.h"

#define TAG "SINE_GEN"
#define PI 3.14159265358979f

typedef struct {
    float frequency;
    float amplitude;
    int   sample_rate;
    int   channels;
    int   bits;
    float phase;        // current phase accumulator
    float phase_inc;    // phase increment per sample
} sine_ctx_t;

static esp_err_t sine_open(audio_element_handle_t self)
{
    sine_ctx_t *ctx = audio_element_getdata(self);
    ctx->phase     = 0.0f;
    ctx->phase_inc = 2.0f * PI * ctx->frequency / ctx->sample_rate;
    return ESP_OK;
}

static esp_err_t sine_close(audio_element_handle_t self)
{
    return ESP_OK;
}

static esp_err_t sine_destroy(audio_element_handle_t self)
{
    sine_ctx_t *ctx = audio_element_getdata(self);
    free(ctx);
    return ESP_OK;
}

static int sine_read(audio_element_handle_t self, char *buf, int len, 
                     TickType_t wait_time, void *context)
{
    sine_ctx_t *ctx = audio_element_getdata(self);

    int samples_per_channel = len / (ctx->bits / 8) / ctx->channels;
    int16_t *out = (int16_t *)buf;
    float amp = ctx->amplitude * 32767.0f;

    for (int i = 0; i < samples_per_channel; i++) {
        int16_t sample = (int16_t)(sinf(ctx->phase) * amp);
        ctx->phase += ctx->phase_inc;
        if (ctx->phase >= 2.0f * PI) ctx->phase -= 2.0f * PI;
        for (int ch = 0; ch < ctx->channels; ch++) {
            *out++ = sample;
        }
    }

    int delay_ms = (samples_per_channel * 1000) / ctx->sample_rate;
    vTaskDelay(pdMS_TO_TICKS(delay_ms > 0 ? delay_ms : 1));

    return len;
}

audio_element_handle_t sine_generator_init(sine_generator_cfg_t *cfg)
{
    sine_ctx_t *ctx = audio_calloc(1, sizeof(sine_ctx_t));
    ctx->frequency   = cfg->frequency;
    ctx->amplitude   = cfg->amplitude;
    ctx->sample_rate = cfg->sample_rate;
    ctx->channels    = cfg->channels;
    ctx->bits        = cfg->bits;

    audio_element_cfg_t el_cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    el_cfg.open       = sine_open;
    el_cfg.close      = sine_close;
    el_cfg.destroy    = sine_destroy;
    el_cfg.read       = sine_read;   // ← was process, now read
    el_cfg.process    = NULL;        // ← explicitly clear process
    el_cfg.tag        = "sine";
    el_cfg.task_stack = 4096;
    el_cfg.buffer_len = 2048;
    el_cfg.task_prio  = 5;

    audio_element_handle_t el = audio_element_init(&el_cfg);
    audio_element_setdata(el, ctx);

    // set stream info so i2s knows the format
    audio_element_info_t info = {
        .sample_rates = cfg->sample_rate,
        .channels     = cfg->channels,
        .bits         = cfg->bits,
    };
    audio_element_setinfo(el, &info);

    return el;
}