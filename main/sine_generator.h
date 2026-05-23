#pragma once
#include "audio_element.h"

typedef struct {
    float frequency;   // Hz, e.g. 440.0
    float amplitude;   // 0.0 to 1.0
    int   sample_rate; // e.g. 44100
    int   channels;    // 1 or 2
    int   bits;        // 16
} sine_generator_cfg_t;

#define SINE_GENERATOR_DEFAULT_CONFIG() { \
    .frequency   = 440.0f,               \
    .amplitude   = 0.5f,                 \
    .sample_rate = 44100,                \
    .channels    = 2,                    \
    .bits        = 16,                   \
}

audio_element_handle_t sine_generator_init(sine_generator_cfg_t *cfg);