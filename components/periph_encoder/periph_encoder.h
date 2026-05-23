#pragma once
#include "esp_peripherals.h"

#define PERIPH_ID_ENCODER       20
#define PERIPH_ENCODER_CW        1
#define PERIPH_ENCODER_CCW       2

typedef struct {
    int gpio_a;
    int gpio_b;
    int step;
} periph_encoder_cfg_t;

esp_periph_handle_t periph_encoder_init(periph_encoder_cfg_t *cfg);