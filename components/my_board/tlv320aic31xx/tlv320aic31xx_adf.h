/*
 * TLV320AIC31xx – ESP-ADF HAL driver header
 *
 * Register map derived from the upstream Linux ALSA driver
 * (SPDX-License-Identifier: GPL-2.0, Texas Instruments).
 * ESP-ADF glue: MIT / Public Domain.
 */

#pragma once

#include "audio_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Register map: AIC_REG(page, reg) = page*128 + reg ─────────────────── */
#define AIC_REG(page, reg)    ((page) * 128 + (reg))

/* Page 0 */
#define AIC_PAGECTL           AIC_REG(0,  0)
#define AIC_RESET             AIC_REG(0,  1)
#define AIC_CLKMUX            AIC_REG(0,  4)
#define AIC_PLLPR             AIC_REG(0,  5)  /* P and R values */
#define AIC_PLLJ              AIC_REG(0,  6)
#define AIC_PLLDMSB           AIC_REG(0,  7)
#define AIC_PLLDLSB           AIC_REG(0,  8)
#define AIC_NDAC              AIC_REG(0, 11)
#define AIC_MDAC              AIC_REG(0, 12)
#define AIC_DOSRMSB           AIC_REG(0, 13)
#define AIC_DOSRLSB           AIC_REG(0, 14)
#define AIC_IFACE1            AIC_REG(0, 27)  /* Audio interface format */
#define AIC_DACSETUP          AIC_REG(0, 63)
#define AIC_DACMUTE           AIC_REG(0, 64)
#define AIC_LDACVOL           AIC_REG(0, 65)
#define AIC_RDACVOL           AIC_REG(0, 66)

/* Page 1 */
#define AIC_HPDRIVER          AIC_REG(1, 31)
#define AIC_SPKAMP            AIC_REG(1, 32)
#define AIC_HPPOP             AIC_REG(1, 33)
#define AIC_DACMIXERROUTE     AIC_REG(1, 35)
#define AIC_LANALOGHPL        AIC_REG(1, 36)
#define AIC_RANALOGHPR        AIC_REG(1, 37)
#define AIC_LANALOGSPL        AIC_REG(1, 38)
#define AIC_RANALOGSPR        AIC_REG(1, 39)
#define AIC_HPLGAIN           AIC_REG(1, 40)
#define AIC_HPRGAIN           AIC_REG(1, 41)
#define AIC_SPLGAIN           AIC_REG(1, 42)
#define AIC_SPRGAIN           AIC_REG(1, 43)

/* ── CLKMUX field values ────────────────────────────────────────────────── */
#define AIC_PLL_CLKIN_BCLK    (0x01 << 2)   /* PLL source = BCLK */
#define AIC_CODEC_CLKIN_PLL   (0x03 << 0)   /* Codec clock = PLL output */

/* ── ADF HAL handle (defined in .c, declared here for board.c) ─────────── */
extern audio_hal_func_t AUDIO_TLV320AIC31XX_DEFAULT_HANDLE;

/* ── Public API ─────────────────────────────────────────────────────────── */
esp_err_t tlv320_init(audio_hal_codec_config_t *cfg);
esp_err_t tlv320_deinit(void);
esp_err_t tlv320_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);
esp_err_t tlv320_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface);
esp_err_t tlv320_set_mute(bool mute);
esp_err_t tlv320_set_volume(int volume);
esp_err_t tlv320_get_volume(int *volume);

#ifdef __cplusplus
}
#endif
