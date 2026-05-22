/*
 * TLV320AIC31xx – ESP-ADF HAL driver
 *
 * I2C wiring comes from get_i2c_pins() (board_pins_config.c), matching
 * the pattern used by every built-in ADF codec driver (ES8388, ES8311…).
 *
 * Clock configuration for 44 100 Hz, 16-bit stereo, I2S slave:
 *   BCLK = 44100 × 32 × 2 = 2 822 400 Hz  (driven by ESP32 I2S peripheral)
 *   PLL source = BCLK
 *   Values from Linux aic31xx_divs[] row {mclk_p=2822400, rate=44100}:
 *     P=1, R=2, J=32, D=0  → PLL_out = 2822400×2×32 = 180 633 600 Hz
 *     NDAC=8, MDAC=2, DOSR=128
 *     DAC_Fs = 180 633 600 / (8×2×128) = 44 100 Hz ✓
 */

#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

/* ADF board layer – provides get_i2c_pins() */
#include "board.h"

#include "tlv320aic31xx_adf.h"

static const char *TAG = "tlv320";

#define AIC_I2C_PORT    I2C_NUM_0
#define AIC_I2C_ADDR    0x18   /* ADDR pin tied to GND; use 0x19 if tied to VDD */
#define AIC_I2C_FREQ    100000

/* ── ADF HAL handle ──────────────────────────────────────────────────────── */
audio_hal_func_t AUDIO_TLV320AIC31XX_DEFAULT_HANDLE = {
    .audio_codec_initialize   = tlv320_init,
    .audio_codec_deinitialize = tlv320_deinit,
    .audio_codec_ctrl         = tlv320_ctrl_state,
    .audio_codec_config_iface = tlv320_config_i2s,
    .audio_codec_set_mute     = tlv320_set_mute,
    .audio_codec_set_volume   = tlv320_set_volume,
    .audio_codec_get_volume   = tlv320_get_volume,
};

/* ── I2C helpers ─────────────────────────────────────────────────────────── */

// static esp_err_t i2c_master_init(void)
// {
//     i2c_config_t cfg = {
//         .mode             = I2C_MODE_MASTER,
//         .sda_pullup_en    = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en    = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = AIC_I2C_FREQ,
//     };
//     /* Delegate pin lookup to the board layer – identical to ES8388 driver */
//     esp_err_t ret = get_i2c_pins(AIC_I2C_PORT, &cfg);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "get_i2c_pins failed (%s)", esp_err_to_name(ret));
//         return ret;
//     }
//     ESP_LOGI(TAG, "I2C SDA=%d SCL=%d", cfg.sda_io_num, cfg.scl_io_num);

//     i2c_param_config(AIC_I2C_PORT, &cfg);
//     ret = i2c_driver_install(AIC_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
//     if (ret == ESP_ERR_INVALID_STATE) {
//         /* Driver already installed by another component – that's fine */
//         ret = ESP_OK;
//     }
//     return ret;
// }

/*
 * Write one register.  The TLV320AIC31xx uses a paged register map;
 * we switch pages by writing register 0x00 on the current page first.
 */
static esp_err_t aic_write(uint16_t reg, uint8_t val)
{
    uint8_t page    = (uint8_t)(reg / 128);
    uint8_t reg_off = (uint8_t)(reg % 128);
    esp_err_t ret;

    /* 1. Select page */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00,  true);   /* PAGECTL */
    i2c_master_write_byte(cmd, page,  true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(AIC_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "page select failed reg=0x%03X (%s)", reg, esp_err_to_name(ret));
        return ret;
    }

    /* 2. Write register */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_off, true);
    i2c_master_write_byte(cmd, val,     true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(AIC_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write failed reg=0x%03X val=0x%02X (%s)",
                 reg, val, esp_err_to_name(ret));
    }
    return ret;
}

static uint8_t aic_read(uint16_t reg)
{
    uint8_t page    = (uint8_t)(reg / 128);
    uint8_t reg_off = (uint8_t)(reg % 128);
    uint8_t val     = 0xFF;

    /* Select page */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, page, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(AIC_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    /* Write register address */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_off, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(AIC_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    /* Read */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIC_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(AIC_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return val;
}

/* ── Init ────────────────────────────────────────────────────────────────── */

// esp_err_t tlv320_init(audio_hal_codec_config_t *cfg)
// {
//     ESP_LOGI(TAG, "init - Arduino compatible sequence");

//     ESP_ERROR_CHECK(i2c_master_init());

//     /* ── Hardware reset ───────────────────────────────────────────── */
//     int rst_gpio = 16;   // or get_codec_rst_gpio()
//     if (rst_gpio >= 0) {
//         gpio_config_t io = {
//             .pin_bit_mask = 1ULL << rst_gpio,
//             .mode = GPIO_MODE_OUTPUT,
//             .pull_up_en = GPIO_PULLUP_DISABLE,
//             .pull_down_en = GPIO_PULLDOWN_DISABLE,
//             .intr_type = GPIO_INTR_DISABLE,
//         };
//         gpio_config(&io);
//         gpio_set_level(rst_gpio, 0);
//         vTaskDelay(pdMS_TO_TICKS(10));
//         gpio_set_level(rst_gpio, 1);
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }

//     /* ── Software reset ───────────────────────────────────────────── */
//     aic_write(AIC_RESET, 0x01);
//     vTaskDelay(pdMS_TO_TICKS(20));

//     /* ───────────────────────────────────────────────────────────────
//      * 1. Audio interface: I2S, 16‑bit, slave
//      *    (Page 0, reg 0x1B = 0x00)
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_IFACE1, 0x00);
//     aic_write(AIC_PLLPR,   0x12);   // P=1, R=2, power off
//     aic_write(AIC_PLLJ,    0x2C);   // J=44
//     aic_write(AIC_PLLDMSB, 0x00);
//     aic_write(AIC_PLLDLSB, 0x00);
//     aic_write(AIC_PLLPR,   0x92);   // power on

//     /* ───────────────────────────────────────────────────────────────
//      * 2. Clock mux: PLL source = BCLK, CODEC_CLKIN = PLL output
//      *    (Page 0, reg 0x04 = 0x07)
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_CLKMUX, 0x07);

//     /* ───────────────────────────────────────────────────────────────
//      * 3. PLL settings for 44.1 kHz (J=44, P=1, R=2, D=0)
//      *    Page 0, reg 0x06 = 0x2C (PLLJ)
//      *    Page 0, reg 0x08 = 0x00 (PLLDLSB)
//      *    Page 0, reg 0x07 = 0x00 (PLLDMSB)
//      *    Page 0, reg 0x05 = 0x12 (PLLPR: P=1,R=2, power off)
//      *    Page 0, reg 0x05 = 0x92 (PLLPR: power on)
//      * ─────────────────────────────────────────────────────────────── */
//     vTaskDelay(pdMS_TO_TICKS(15));   // wait for PLL lock

//     /* ───────────────────────────────────────────────────────────────
//      * 4. DAC clock dividers (NDAC=8, MDAC=2, DOSR=128)
//      *    Page 0, reg 0x0B = 0x88
//      *    Page 0, reg 0x0C = 0x82
//      *    Page 0, reg 0x0D = 0x00
//      *    Page 0, reg 0x0E = 0x80
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_NDAC,    0x88);
//     aic_write(AIC_MDAC,    0x82);
//     aic_write(AIC_DOSRMSB, 0x00);
//     aic_write(AIC_DOSRLSB, 128);

//     /* ───────────────────────────────────────────────────────────────
//      * 5. DAC data path (enable both channels, left→left, right→right)
//      *    Page 0, reg 0x3F = 0xD4  (matches Arduino log)
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_DACSETUP, 0xD4);

//     /* ───────────────────────────────────────────────────────────────
//      * 6. Digital volume and mute
//      *    Page 0, reg 0x40 = 0x00 (unmute)
//      *    Page 0, reg 0x41 = 0x18 (left DAC gain +12 dB)
//      *    Page 0, reg 0x42 = 0x18 (right DAC gain +12 dB)
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_DACMUTE,  0x00);
//     // aic_write(AIC_LDACVOL,  0x18);
//     // aic_write(AIC_RDACVOL,  0x18);
//     aic_write(AIC_LDACVOL, 0x00);
//     aic_write(AIC_RDACVOL, 0x00);

//     /* ───────────────────────────────────────────────────────────────
//      * 7. Switch to Page 1 for analog mixer / output registers
//      * ─────────────────────────────────────────────────────────────── */
//     aic_write(AIC_PAGECTL, 1);

//     /* DAC mixer routing: DAC_L → HPL vol, DAC_R → HPR vol (0x44) */
//     aic_write(AIC_DACMIXERROUTE, 0x44);

//     /* Headphone pop‑suppression ramp (same as original) */
//     aic_write(AIC_HPPOP, 0x4E);
//     /* Headphone driver: enable both channels (0xC4) */
//     aic_write(AIC_HPDRIVER, 0xC4);
//     /* Headphone analog volume: -32 dB (0x40) */
//     /* Headphone gain: +3 dB (0x06) */
//     // aic_write(AIC_HPLGAIN, 0x06);
//     // aic_write(AIC_HPRGAIN, 0x06);

//     // aic_write(AIC_HPLGAIN, 2); // <-- EARRAPE
//     // aic_write(AIC_HPRGAIN, 2);
//     // aic_write(AIC_LANALOGHPL, 10);
//     // aic_write(AIC_RANALOGHPR, 10);

//     aic_write(AIC_HPDRIVER, 0xC0);
//     // 1 dB gain, unmuted:  (1 << 3) | 0x04 = 0x0C
//     // 2 dB gain, unmuted:  (2 << 3) | 0x04 = 0x14
//     // 3 dB gain, unmuted:  (3 << 3) | 0x04 = 0x1C
//     aic_write(AIC_HPLGAIN, 0x14);
//     aic_write(AIC_HPRGAIN, 0x14);
//     aic_write(AIC_LANALOGHPL, 0x00); // <-- la
//     aic_write(AIC_RANALOGHPR, 0x00);
//     aic_write(AIC_HPCONTROL, 0x0C);

//     /* Speaker amplifier: enable, class‑D gain 6 dB (0x86) */
//     aic_write(AIC_SPKAMP, 0x86);

//     // /* Speaker analog volume: 0 dB (0x00) – not used in the log, keep 0 */
//     // aic_write(AIC_LANALOGSPL, 0x00);
//     // aic_write(AIC_RANALOGSPR, 0x00);

//     /* Speaker gain: +2.5 dB (0x05) */
//     aic_write(AIC_SPLGAIN, 0x05);
//     aic_write(AIC_SPRGAIN, 0x05);


// // aic_write(AIC_SPKAMP, 0x80);        // Enable speaker amp, 0 dB gain (instead of 0x86)
// // aic_write(AIC_SPLGAIN, 0x00);
// // aic_write(AIC_SPRGAIN, 0x00);
// // aic_write(AIC_LANALOGSPL, 0x7F);    // Start at max attenuation (-63.5 dB)
// // aic_write(AIC_RANALOGSPR, 0x7F);
// aic_write(AIC_SPKAMP, 0x80);
// // aic_write(AIC_SPLGAIN, 0x00);
// // aic_write(AIC_SPRGAIN, 0x00);
// // Set speaker analog volume to -32 dB (0x40) – audible but not deafening
// aic_write(AIC_LANALOGSPL, 0x20);
// aic_write(AIC_RANALOGSPR, 0x20); // <-- la

//     /* Return to Page 0 (optional) */
//     aic_write(AIC_PAGECTL, 0);

//     ESP_LOGI(TAG, "init done (Arduino compatible)");
//     return ESP_OK;
// }

// esp_err_t tlv320_init(audio_hal_codec_config_t *cfg)
// {
//     ESP_LOGI(TAG, "init");

//     ESP_ERROR_CHECK(i2c_master_init());

//     /* ── Hardware reset ─────────────────────────────────────────────────────
//      * Mirrors the Linux driver sequence (aic31xx_i2c_probe → gpio_reset):
//      *   drive RST low for ≥10 ms  (assert)
//      *   drive RST high            (release)
//      *   wait ≥1 ms before first I2C access (datasheet §5.2)
//      *
//      * get_codec_rst_gpio() returns -1 if no reset pin is wired; in that
//      * case we fall through to the software reset below.
//      * ──────────────────────────────────────────────────────────────────── */
//     // int rst_gpio = get_codec_rst_gpio();
//     int rst_gpio = 16;
//     if (rst_gpio >= 0) {
//         ESP_LOGI(TAG, "HW reset on GPIO %d", rst_gpio);
//         gpio_config_t io = {
//             .pin_bit_mask = 1ULL << rst_gpio,
//             .mode         = GPIO_MODE_OUTPUT,
//             .pull_up_en   = GPIO_PULLUP_DISABLE,
//             .pull_down_en = GPIO_PULLDOWN_DISABLE,
//             .intr_type    = GPIO_INTR_DISABLE,
//         };
//         gpio_config(&io);
//         gpio_set_level(rst_gpio, 0);          /* assert reset */
//         vTaskDelay(pdMS_TO_TICKS(10));
//         gpio_set_level(rst_gpio, 1);          /* release reset */
//         vTaskDelay(pdMS_TO_TICKS(2));         /* ≥1 ms settling time */
//     }

//     /* ── Software reset (clears all registers to defaults) ──────────────── */
//     aic_write(AIC_RESET, 0x01);
//     vTaskDelay(pdMS_TO_TICKS(10));

//     /* ── Clock: BCLK → PLL → codec_clk ─────────────────────────────────── */
//     /*
//      * CLKMUX (page 0, reg 4):
//      *   bits[3:2] PLL_CLKIN  = 01 (BCLK)
//      *   bits[1:0] CODEC_CLKIN = 11 (PLL)
//      */
//     aic_write(AIC_CLKMUX, AIC_PLL_CLKIN_BCLK | AIC_CODEC_CLKIN_PLL);

//     /*
//      * PLLPR (page 0, reg 5):
//      *   bit 7     = PLL power (set later)
//      *   bits[6:4] = P = 1 → 0b001
//      *   bits[3:0] = R = 2 → 0b0010
//      *   → 0b0_001_0010 = 0x12  (powered-off placeholder)
//      *
//      * Source: Linux aic31xx_divs[] {mclk_p=2822400, rate=44100, r=2, j=32, d=0}
//      */
//     aic_write(AIC_PLLPR,   0x12);
//     aic_write(AIC_PLLJ,    32);
//     aic_write(AIC_PLLDMSB, 0x00);
//     aic_write(AIC_PLLDLSB, 0x00);
//     /* Power on PLL: bit7 = 1 */
//     aic_write(AIC_PLLPR,   0x92);   /* 0b1_001_0010 */
//     vTaskDelay(pdMS_TO_TICKS(15));  /* wait for lock */

//     /*
//      * NDAC / MDAC / DOSR
//      *   bit7 = power enable, bits[6:0] = divider value
//      *   Linux table: ndac=8, mdac=2, dosr=128
//      */
//     aic_write(AIC_NDAC,    0x88);   /* powered | 8   */
//     aic_write(AIC_MDAC,    0x82);   /* powered | 2   */
//     aic_write(AIC_DOSRMSB, 0x00);
//     aic_write(AIC_DOSRLSB, 128);

//     /* ── Audio interface: I2S, 16-bit, slave (all zeros = default) ──────── */
//     aic_write(AIC_IFACE1, 0x00);

//     /* ── DAC data path ──────────────────────────────────────────────────── */
//     /*
//      * DACSETUP (page 0, reg 63):
//      *   bits[7:6] = DAC power: 10 = L+R on
//      *   bits[5:4] = Left  DAC channel data: 01 = left
//      *   bits[3:2] = Right DAC channel data: 01 = right
//      *   → 0b10_01_01_00 = 0x94
//      */
//     aic_write(AIC_DACSETUP, 0x94);

//     /*
//      * DACMIXERROUTE (page 1, reg 35):
//      *   bits[7:6] = DAC_L → HPL mixer: 10 = left DAC
//      *   bits[3:2] = DAC_R → HPR mixer: 10 = right DAC
//      *   → 0b10_00_10_00 = 0x88
//      *   Also enables SPL path on same bits if SPL shares mixer
//      *   Use 0x44 to route via analog volume (DAC_L → HPL vol, DAC_R → HPR vol)
//      */
//     aic_write(AIC_DACMIXERROUTE, 0x44);

//     /* Analog volume to HP outputs: 0x00 = 0 dB, 0x7F = muted */
//     aic_write(AIC_LANALOGHPL,  0x00);
//     aic_write(AIC_RANALOGHPR,  0x00);
//     aic_write(AIC_LANALOGSPL,  0x00);
//     aic_write(AIC_RANALOGSPR,  0x00);

//     /* ── Headphone amp ──────────────────────────────────────────────────── */
//     /* HPPOP: slow power-up ramp to suppress pop noise */
//     aic_write(AIC_HPPOP,    0x4E);
//     /* HPDRIVER: bit7=HPL_EN, bit6=HPR_EN */
//     aic_write(AIC_HPDRIVER, 0xC0);
//     /* HP driver gain = 0 dB, mute bit cleared */
//     aic_write(AIC_HPLGAIN,  0x00);
//     aic_write(AIC_HPRGAIN,  0x00);

//     /* ── Speaker amp ────────────────────────────────────────────────────── */
//     /* SPKAMP: bit7=SPL_EN, bits[2:1]=6 dB class-D gain (default) */
//     aic_write(AIC_SPKAMP,   0x86);
//     aic_write(AIC_SPLGAIN,  0x00);
//     aic_write(AIC_SPRGAIN,  0x00);

//     /* ── Digital volume and unmute ──────────────────────────────────────── */
//     aic_write(AIC_LDACVOL, 0x00);   /* 0 dB */
//     aic_write(AIC_RDACVOL, 0x00);   /* 0 dB */
//     /*
//      * DACMUTE (page 0, reg 64):
//      *   bits[3:2] mute: 00 = unmuted
//      *   bits[1:0] soft-step: 00 = 1 step per Fs
//      */
//     aic_write(AIC_DACMUTE,  0x00);

//     ESP_LOGI(TAG, "init done");
//     return ESP_OK;
// }

esp_err_t tlv320_deinit(void)
{
    aic_write(AIC_DACMUTE, 0x0C);   /* mute L+R */
    return ESP_OK;
}

/* ── Ctrl / config stubs (ADF calls these; config is fixed in init) ──────── */

esp_err_t tlv320_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    return ESP_OK;
}

esp_err_t tlv320_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    return ESP_OK;
}

/* ── Volume / mute ───────────────────────────────────────────────────────── */

esp_err_t tlv320_set_mute(bool mute)
{
    /* DACMUTE bits[3:2]: 11=muted, 00=unmuted */
    // return aic_write(AIC_DACMUTE, mute ? 0x0C : 0x00);
    return ESP_OK;
}

// esp_err_t tlv320_set_volume(int volume)
// {
//     /*
//      * Digital volume register: two's complement, 0.5 dB/step.
//      *   0x00 =  0 dB (loudest)
//      *   0x81 = -63.5 dB
//      * Map linear 0..100 → 0 dB..-40 dB (reg 0x00..0x50)
//      */
//     if (volume < 0)   volume = 0;
//     if (volume > 100) volume = 100;
//     uint8_t reg_val = (uint8_t)((100 - volume) * 0x50 / 100);
//     aic_write(AIC_LDACVOL, reg_val);
//     return aic_write(AIC_RDACVOL, reg_val);
// }

// esp_err_t tlv320_set_volume(int volume)
// {
//     if (volume < 0) volume = 0;
//     if (volume > 100) volume = 100;

//     // Map 0..100 -> 0x7F..0x00 (0.5 dB steps)
//     uint8_t reg_val = ((100 - volume) * 0x7F) / 100;

//     // Only control headphone analog volume
//     aic_write(AIC_LANALOGHPL, 5);
//     aic_write(AIC_RANALOGHPR, 5);

//     ESP_LOGI(TAG, "Headphone analog volume set to 0x%02X (volume %d%%)", reg_val, volume);
//     return ESP_OK;
// }
esp_err_t tlv320_set_volume(int volume)
{
    // if (volume < 0)   volume = 0;
    // if (volume > 100) volume = 100;
    // uint8_t reg_val = (uint8_t)(((100 - volume) * 0x7F) / 100);

    // /* Disable HP driver first */
    // aic_write(AIC_HPDRIVER, 0x00);
    // aic_write(AIC_LANALOGHPL, 20);
    // aic_write(AIC_RANALOGHPR, 20);
    // aic_write(AIC_HPDRIVER, 0xC4);

    // /* --- DIAGNOSTIC: read back to confirm the write landed --- */
    // uint8_t rb = aic_read(AIC_LANALOGHPL);
    // ESP_LOGE(TAG, "LANALOGHPL wrote 0x%02X, read back 0x%02X", reg_val, rb);

    // aic_write(AIC_LANALOGSPL, 45);
    // aic_write(AIC_RANALOGSPR, 45);
    return ESP_OK;
}
// esp_err_t tlv320_set_volume(int volume)
// {
//     aic_write(AIC_HPDRIVER, 0x00);
//     aic_write(AIC_LANALOGHPL, 0);
//     aic_write(AIC_RANALOGHPR, 0);
//     aic_write(AIC_HPDRIVER, 0xC4);
//     aic_write(AIC_LANALOGSPL, 45);
//     aic_write(AIC_RANALOGSPR, 45);
//     return ESP_OK;
// }
// esp_err_t tlv320_set_volume(int volume)
// {
//     if (volume < 0) volume = 0;
//     if (volume > 100) volume = 100;
//     // Map 0..100 to 0x7F..0x00 (0.5 dB steps, 0x7F = -63.5 dB, 0x00 = 0 dB)
//     uint8_t reg_val = ((100 - volume) * 0x7F) / 100;
//     aic_write(AIC_LDACVOL, 0x00);
//     aic_write(AIC_RDACVOL, 0x00);
//     aic_write(AIC_HPLGAIN, 0x00);
//     aic_write(AIC_HPRGAIN, 0x00);
//     aic_write(AIC_LANALOGHPL, 10);
//     aic_write(AIC_RANALOGHPR, 10);
//     ESP_LOGE(TAG, "volume set");
//     return ESP_OK;
// }
// esp_err_t tlv320_set_volume(int volume)
// {
//     if (volume < 0) volume = 0;
//     if (volume > 100) volume = 100;

//     // Map 0..100 to 0x7F..0x00 (0.5 dB steps, 0x7F = -63.5 dB, 0x00 = 0 dB)
//     uint8_t reg_val = ((100 - volume) * 0x7F) / 100;

//     // Headphone analog volume
//     aic_write(AIC_LANALOGHPL, reg_val);
//     aic_write(AIC_RANALOGHPR, reg_val);

//     // Speaker analog volume (critical for boards where speaker and HP share output)
//     aic_write(AIC_LANALOGSPL, reg_val);
//     aic_write(AIC_RANALOGSPR, reg_val);

//     // Ensure digital volume is 0 dB (already set in init)
//     aic_write(AIC_LDACVOL, 0x00);
//     aic_write(AIC_RDACVOL, 0x00);

//     // Set headphone gain to 0 dB (already in init, but safe to repeat)
//     aic_write(AIC_HPLGAIN, 0x00);
//     aic_write(AIC_HPRGAIN, 0x00);

//     // Reduce speaker gain to 0 dB (instead of +2.5 dB)
//     aic_write(AIC_SPLGAIN, 0x00);
//     aic_write(AIC_SPRGAIN, 0x00);

//     ESP_LOGI(TAG, "Volume set to %d%% -> analog volume 0x%02X", volume, reg_val);
//     return ESP_OK;
// }

esp_err_t tlv320_get_volume(int *volume)
{
    *volume = 80;   /* read-back omitted for brevity */
    return ESP_OK;
}

static struct {
    i2c_bus_handle_t i2c_handle;
} tlv320_handle;

uint8_t current_page = 0;

// Extract page and register address from AIC31XX_REG macro
uint8_t tlv320_get_page(uint16_t reg) {
    return reg / 128;
}

uint8_t tlv_320_get_register(uint16_t reg) {
    return reg % 128;
}

const char* tlv_320_lookup_register_name(uint16_t address) {
    for (size_t i = 0; i < sizeof(registerTable) / sizeof(reg_name_t); i++) {
        if (registerTable[i].reg == address) {
            return registerTable[i].name;
        }
    }
    return "Unknown Register";
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = AIC_I2C_FREQ,
    };
    /* Delegate pin lookup to the board layer – identical to ES8388 driver */
    esp_err_t ret = get_i2c_pins(AIC_I2C_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "get_i2c_pins failed (%s)", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C SDA=%d SCL=%d", cfg.sda_io_num, cfg.scl_io_num);
    tlv320_handle.i2c_handle = i2c_bus_create(AIC_I2C_PORT, &cfg);
    return ret;
}

static esp_err_t tlv320_write_reg(uint8_t reg_addr, uint8_t data)
{
    return i2c_bus_write_bytes(tlv320_handle.i2c_handle, TLV320AIC31XX_I2C_ADDRESS, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

// Function to set the active page
esp_err_t tlv_320_set_page(uint8_t page) {
    if (current_page == page)
        return ESP_OK;
    ESP_LOGW(TAG, "INFO Set Page: %d", page);
    current_page = page;
    return tlv320_write_reg(PAGE_CTRL_REGISTER, page);
}

esp_err_t tlv320_init(audio_hal_codec_config_t *cfg)
{
    ESP_LOGI(TAG, "init - Arduino compatible sequence");

    ESP_ERROR_CHECK(i2c_master_init());
    tlv_320_set_page(1);
    vTaskDelay(pdMS_TO_TICKS(100));
    tlv_320_set_page(0);

    /* ── Hardware reset ───────────────────────────────────────────── */
    int rst_gpio = 16;   // or get_codec_rst_gpio()
    if (rst_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << rst_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* ── Software reset ───────────────────────────────────────────── */
    aic_write(AIC_RESET, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* ───────────────────────────────────────────────────────────────
     * 1. Audio interface: I2S, 16‑bit, slave
     *    (Page 0, reg 0x1B = 0x00)
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_IFACE1, 0x00);
    aic_write(AIC_PLLPR,   0x12);   // P=1, R=2, power off
    aic_write(AIC_PLLJ,    0x2C);   // J=44
    aic_write(AIC_PLLDMSB, 0x00);
    aic_write(AIC_PLLDLSB, 0x00);
    aic_write(AIC_PLLPR,   0x92);   // power on

    /* ───────────────────────────────────────────────────────────────
     * 2. Clock mux: PLL source = BCLK, CODEC_CLKIN = PLL output
     *    (Page 0, reg 0x04 = 0x07)
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_CLKMUX, 0x07);

    /* ───────────────────────────────────────────────────────────────
     * 3. PLL settings for 44.1 kHz (J=44, P=1, R=2, D=0)
     *    Page 0, reg 0x06 = 0x2C (PLLJ)
     *    Page 0, reg 0x08 = 0x00 (PLLDLSB)
     *    Page 0, reg 0x07 = 0x00 (PLLDMSB)
     *    Page 0, reg 0x05 = 0x12 (PLLPR: P=1,R=2, power off)
     *    Page 0, reg 0x05 = 0x92 (PLLPR: power on)
     * ─────────────────────────────────────────────────────────────── */
    vTaskDelay(pdMS_TO_TICKS(15));   // wait for PLL lock

    /* ───────────────────────────────────────────────────────────────
     * 4. DAC clock dividers (NDAC=8, MDAC=2, DOSR=128)
     *    Page 0, reg 0x0B = 0x88
     *    Page 0, reg 0x0C = 0x82
     *    Page 0, reg 0x0D = 0x00
     *    Page 0, reg 0x0E = 0x80
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_NDAC,    0x88);
    aic_write(AIC_MDAC,    0x82);
    aic_write(AIC_DOSRMSB, 0x00);
    aic_write(AIC_DOSRLSB, 128);

    /* ───────────────────────────────────────────────────────────────
     * 5. DAC data path (enable both channels, left→left, right→right)
     *    Page 0, reg 0x3F = 0xD4  (matches Arduino log)
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_DACSETUP, 0xD4);

    /* ───────────────────────────────────────────────────────────────
     * 6. Digital volume and mute
     *    Page 0, reg 0x40 = 0x00 (unmute)
     *    Page 0, reg 0x41 = 0x18 (left DAC gain +12 dB)
     *    Page 0, reg 0x42 = 0x18 (right DAC gain +12 dB)
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_DACMUTE,  0x00);
    // aic_write(AIC_LDACVOL,  0x18);
    // aic_write(AIC_RDACVOL,  0x18);
    aic_write(AIC_LDACVOL, 0x00);
    aic_write(AIC_RDACVOL, 0x00);

    /* ───────────────────────────────────────────────────────────────
     * 7. Switch to Page 1 for analog mixer / output registers
     * ─────────────────────────────────────────────────────────────── */
    aic_write(AIC_PAGECTL, 1);

    /* DAC mixer routing: DAC_L → HPL vol, DAC_R → HPR vol (0x44) */
    aic_write(AIC_DACMIXERROUTE, 0x44);

    /* Headphone pop‑suppression ramp (same as original) */
    aic_write(AIC_HPPOP, 0x4E);
    /* Headphone driver: enable both channels (0xC4) */
    aic_write(AIC_HPDRIVER, 0xC4);
    /* Headphone analog volume: -32 dB (0x40) */
    /* Headphone gain: +3 dB (0x06) */
    // aic_write(AIC_HPLGAIN, 0x06);
    // aic_write(AIC_HPRGAIN, 0x06);

    // aic_write(AIC_HPLGAIN, 2); // <-- EARRAPE
    // aic_write(AIC_HPRGAIN, 2);
    // aic_write(AIC_LANALOGHPL, 10);
    // aic_write(AIC_RANALOGHPR, 10);

    aic_write(AIC_HPDRIVER, 0xC0);
    // 1 dB gain, unmuted:  (1 << 3) | 0x04 = 0x0C
    // 2 dB gain, unmuted:  (2 << 3) | 0x04 = 0x14
    // 3 dB gain, unmuted:  (3 << 3) | 0x04 = 0x1C
    aic_write(AIC_HPLGAIN, 0x14);
    aic_write(AIC_HPRGAIN, 0x14);
    aic_write(AIC_LANALOGHPL, 0x00); // <-- la
    aic_write(AIC_RANALOGHPR, 0x00);
    aic_write(AIC_HPCONTROL, 0x0C);

    /* Speaker amplifier: enable, class‑D gain 6 dB (0x86) */
    aic_write(AIC_SPKAMP, 0x86);

    // /* Speaker analog volume: 0 dB (0x00) – not used in the log, keep 0 */
    // aic_write(AIC_LANALOGSPL, 0x00);
    // aic_write(AIC_RANALOGSPR, 0x00);

    /* Speaker gain: +2.5 dB (0x05) */
    aic_write(AIC_SPLGAIN, 0x05);
    aic_write(AIC_SPRGAIN, 0x05);


// aic_write(AIC_SPKAMP, 0x80);        // Enable speaker amp, 0 dB gain (instead of 0x86)
// aic_write(AIC_SPLGAIN, 0x00);
// aic_write(AIC_SPRGAIN, 0x00);
// aic_write(AIC_LANALOGSPL, 0x7F);    // Start at max attenuation (-63.5 dB)
// aic_write(AIC_RANALOGSPR, 0x7F);
aic_write(AIC_SPKAMP, 0x80);
// aic_write(AIC_SPLGAIN, 0x00);
// aic_write(AIC_SPRGAIN, 0x00);
// Set speaker analog volume to -32 dB (0x40) – audible but not deafening
aic_write(AIC_LANALOGSPL, 0x20);
aic_write(AIC_RANALOGSPR, 0x20); // <-- la

    /* Return to Page 0 (optional) */
    aic_write(AIC_PAGECTL, 0);

    ESP_LOGI(TAG, "init done (Arduino compatible)");
    return ESP_OK;
}