#include <string.h>
#include <math.h>
#include "esp_log.h"
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

esp_err_t tlv_320_set_page(uint8_t page);

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

esp_err_t tlv320_deinit(void)
{
    // aic_write(AIC_DACMUTE, 0x0C);   /* mute L+R */
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

esp_err_t tlv320_set_mute(bool mute)
{
    /* DACMUTE bits[3:2]: 11=muted, 00=unmuted */
    // return aic_write(AIC_DACMUTE, mute ? 0x0C : 0x00);
    return ESP_OK;
}

esp_err_t tlv320_set_volume(int volume)
{
    return ESP_OK;
}

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


static int tlv320_write_reg(uint8_t reg, uint8_t value) {
    uint8_t page = tlv320_get_page(reg);
    tlv_320_set_page(page);
    uint8_t reg_addr = tlv_320_get_register(reg);
    return i2c_bus_write_bytes(tlv320_handle.i2c_handle,
        (TLV320AIC31XX_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        &reg_addr, sizeof(reg_addr), &value, sizeof(value));
}

static int tlv320_read_reg(uint8_t reg)
{
    uint8_t page = tlv320_get_page(reg);
    uint8_t value = 0;
    tlv_320_set_page(page);
    uint8_t reg_addr = tlv_320_get_register(reg);
    uint8_t data;
    esp_err_t ret = i2c_bus_read_bytes(tlv320_handle.i2c_handle,
        (TLV320AIC31XX_I2C_ADDRESS << 1) | I2C_MASTER_READ,
        &reg_addr, sizeof(reg_addr), &data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to tlv320 read 0x%x reg", reg_addr);
    }
    return (int)data;
}

// Function to set the active page
esp_err_t tlv_320_set_page(uint8_t page) {
    if (current_page == page)
        return ESP_OK;
    ESP_LOGW(TAG, "INFO Set Page: %d", page);
    current_page = page;
    uint8_t reg = PAGE_CTRL_REGISTER;
    return i2c_bus_write_bytes(tlv320_handle.i2c_handle,
        (TLV320AIC31XX_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        &reg, sizeof(PAGE_CTRL_REGISTER), &page, sizeof(page));
}

// Function to read, modify, and write back specific bits in a register
void tlv320_modify_reg(uint16_t reg, uint8_t mask, uint8_t value) {
    uint8_t currentValue = tlv320_read_reg(reg);
    uint8_t shift = 0;

    // determine left shift bit count from mask
    while (((mask >> shift) & 1) == 0 && shift < 8) {
        shift++;
    }

    // Shift the value into position
    uint8_t shiftedValue = (value << shift) & mask;
    uint8_t newValue = (currentValue & ~mask) | shiftedValue;
    tlv320_write_reg(reg, newValue);
}

uint8_t tlv320_convert_dac_volume_to_register_value(float dB) {
    // Clamp dB to valid range
    if (dB > 24.0f) dB = 24.0f; // Max volume: +24 dB
    if (dB < -63.5f) dB = -63.5f; // Min volume: -63.5 dB

    // Convert dB to register value (0.5 dB steps)
    int16_t value = dB * 2; // Multiply by 2 to handle 0.5 dB steps

    // Adjust for the unsigned register format:
    // - Positive dB values are mapped directly (0x00 to 0x30 for 0 dB to +24 dB).
    // - Negative dB values are mapped to 2's complement (0xFF to 0x82 for -0.5 dB to -63 dB).
    uint8_t regValue = (value >= 0) ? value : (0x100 + value);

    // Ensure that reserved values (0x80) are not used
    if (regValue == 0x80) regValue = 0x81; // Adjust to closest valid value (-63.5 dB)

    return regValue;
}

uint8_t tlv320_convert_adc_volume_to_register_value(float dB) {
    // Clamp dB to valid range
    if (dB > 20.0f) dB = 20.0f;   // Max volume: +20 dB
    if (dB < -12.0f) dB = -12.0f; // Min volume: -12 dB

    uint8_t regValue;

    if (dB < 0.0f) {
        // Negative dB: map -12.0 dB to 0x68, -0.5 dB to 0x7F
        regValue = 0x68 + ((dB + 12.0f) * 2);
    } else {
        // Non-negative dB: map 0.0 dB to 0x00, +20.0 dB to 0x28
        regValue = dB * 2;
    }

    return regValue;
}

uint8_t tlv320_convert_analog_gain_to_register_value(float gainDb) {
    // Clamp gain to valid range
    if (gainDb > analogGainTable[0]) {
        gainDb = analogGainTable[0]; // Maximum gain: 0.0 dB
    } else if (gainDb < analogGainTable[127]) {
	    gainDb = analogGainTable[127]; // Minimum gain: -78.3 dB
    }

    // Find the closest match in the table
    uint8_t closestIndex = 0;
    float smallestDifference = fabs(gainDb - analogGainTable[0]);

    for (uint8_t i = 1; i < sizeof(analogGainTable) / sizeof(float); ++i) {
        float difference = fabs(gainDb - analogGainTable[i]);
        if (difference < smallestDifference) {
            closestIndex = i;
            smallestDifference = difference;
        }
    }

    return closestIndex;
}

uint8_t tlv320_convert_pga_gain_to_register_value(float gainDb) {
    // Clamp gain to the valid range [0, 9]
    if (gainDb < 0.0f) {
        gainDb = 0.0f;
    } else if (gainDb > 9.0f) {
        gainDb = 9.0f;
    }

    // Convert directly to register value
    return gainDb;
}

uint8_t tlv320_convert_mic_pga_gain_to_register_value(float gainDb) {
    // Clamp gain to the valid range [0.0, 59.5]
    if (gainDb < 0.0f) {
        gainDb = 0.0f;
    } else if (gainDb > 59.5f) {
        gainDb = 59.5f;
    }

    return gainDb * 2;
}

void tlv320_set_pll(uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d) {
	tlv320_write_reg(AIC31XX_PLLJ, pll_j);
	tlv320_write_reg(AIC31XX_PLLDLSB, pll_d & 0xff);
	tlv320_write_reg(AIC31XX_PLLDMSB, (pll_d>>8) & 0xff);
	tlv320_modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_R_MASK, pll_r);
	tlv320_modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_P_MASK, pll_p);
}

void tlv320_set_pll_power(bool power) {
	tlv320_modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_POWER_MASK, power ? 1 : 0);
}

void tlv320_set_clk_mux(uint8_t pll_clkin, uint8_t codec_clkin) {
	// The source reference clock for the codec is chosen by programming the CODEC_CLKIN value on page 0 / register 4, bits D1–D0. 
	tlv320_modify_reg(AIC31XX_CLKMUX, AIC31XX_CODEC_CLKIN_MASK, codec_clkin);
	tlv320_modify_reg(AIC31XX_CLKMUX, AIC31XX_PLL_CLKIN_MASK, pll_clkin);
}

void tlv320_set_ndac_val(uint8_t ndac) {
	tlv320_modify_reg(AIC31XX_NDAC, AIC31XX_NDAC_MASK, ndac);
}
void tlv320_set_ndac_power(bool power) {
	tlv320_modify_reg(AIC31XX_NDAC, AIC31XX_NDAC_POWER_MASK, power);
}
void tlv320_set_mdac_val(uint8_t mdac) {
	tlv320_modify_reg(AIC31XX_MDAC, AIC31XX_MDAC_MASK, mdac);
}
void tlv320_set_mdac_power(bool power) {
	tlv320_modify_reg(AIC31XX_MDAC, AIC31XX_MDAC_POWER_MASK, power);
}
void tlv320_set_dosr_val(uint16_t dosr) {
	tlv320_write_reg(AIC31XX_DOSRLSB, dosr & 0xff);
	tlv320_write_reg(AIC31XX_DOSRMSB, (dosr>>8) & 0xff);
}

void tlv320_set_nadc_val(uint8_t nadc) {
	tlv320_modify_reg(AIC31XX_NADC, AIC31XX_NADC_MASK, nadc);
}
void tlv320_set_nadc_power(bool power) {
	tlv320_modify_reg(AIC31XX_NADC, AIC31XX_NADC_POWER_MASK, power);
}
void tlv320_set_madc_val(uint8_t madc) {
	tlv320_modify_reg(AIC31XX_MADC, AIC31XX_MADC_MASK, madc);
}
void tlv320_set_madc_power(bool power) {
	tlv320_modify_reg(AIC31XX_MADC, AIC31XX_MADC_POWER_MASK, power);
}
void tlv320_set_aosr_val(uint16_t aosr) {
	tlv320_write_reg(AIC31XX_AOSR, aosr);
}

void tlv320_set_word_length(uint8_t wordlength) {
	tlv320_modify_reg(AIC31XX_IFACE1, AIC31XX_IFACE1_DATALEN_MASK, wordlength);
}

void tlv320_set_hs_detect_int1(bool enable) {
	tlv320_modify_reg(AIC31XX_INT1CTRL, AIC31XX_HSPLUGDET, enable);
	tlv320_modify_reg(AIC31XX_GPIO1, AIC31XX_GPIO1_FUNC_MASK, AIC31XX_GPIO1_INT1);
}

void tlv320_enable_headset_detect() {
	tlv320_modify_reg(AIC31XX_HSDETECT, AIC31XX_HSD_ENABLE, 0x01);
}

bool tlv320_is_headset_detected() {
	return (tlv320_read_reg(AIC31XX_HSDETECT) & AIC31XX_HSD_TYPE_MASK) != AIC31XX_HSD_NONE;
}


// High-level function to enable and unmute the DAC and route it to the output mixer
void tlv320_enable_dac() {
	tlv320_modify_reg(AIC31XX_DACSETUP, AIC31XX_DAC_POWER_MASK, 0x3);
	tlv320_modify_reg(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACL_MASK, 0x1);
	tlv320_modify_reg(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACR_MASK, 0x1);
}

void tlv320_set_dac_mute(bool mute) {
	if(mute) {
		tlv320_modify_reg(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x3);
	} else {
		// unmute
		tlv320_modify_reg(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x0);
	}
}

void tlv320_set_dac_volume(float left_dB, float right_dB) {
    // Convert dB values to register format
    uint8_t leftRegValue = tlv320_convert_dac_volume_to_register_value(left_dB);
    uint8_t rightRegValue = tlv320_convert_dac_volume_to_register_value(right_dB);

    // Write to left and right DAC volume registers
    tlv320_write_reg(AIC31XX_LDACVOL, leftRegValue);  // Left DAC volume control
    tlv320_write_reg(AIC31XX_RDACVOL, rightRegValue); // Right DAC volume control
}

void tlv320_enable_adc() {
	// power
	tlv320_modify_reg(AIC31XX_ADCSETUP, AIC31XX_ADC_POWER_MASK,0x1);
	// unmute
	tlv320_modify_reg(AIC31XX_ADCFGA, AIC31XX_ADC_MUTE_MASK,0x0);
}

// enable the headphone amplifier
void tlv320_enable_headphone_amp() {
	tlv320_modify_reg(AIC31XX_HPDRIVER, AIC31XX_HPD_POWER_MASK, 0x3);
}

void tlv320_set_headphone_mute(bool mute) {
	tlv320_modify_reg(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
	tlv320_modify_reg(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void tlv320_set_headphone_gain(float left_dB, float right_dB) {
	tlv320_modify_reg(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_GAIN_MASK, tlv320_convert_pga_gain_to_register_value(left_dB));
	tlv320_modify_reg(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_GAIN_MASK, tlv320_convert_pga_gain_to_register_value(right_dB));
}

void tlv320_set_headphone_volume(float left_dB, float right_dB) {
	tlv320_write_reg(AIC31XX_LANALOGHPL, tlv320_convert_analog_gain_to_register_value(left_dB) );
	tlv320_write_reg(AIC31XX_RANALOGHPR, tlv320_convert_analog_gain_to_register_value(right_dB) );
}

void tlv320_set_headphone_performance(uint8_t level) {
	tlv320_modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_PERFORMANCE_MASK, level);
}

void tlv320_set_headphone_line_mode(bool line) {
	tlv320_modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPL_LINE_MASK, line);
	tlv320_modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPR_LINE_MASK, line);
}

void tlv320_enable_speaker_amp() {
	tlv320_modify_reg(AIC31XX_SPKAMP, AIC3100_SPKAMP_POWER_MASK, 0x1);
	// TODO: also support other channel on codecs with stereo amp
}

void tlv320_set_speaker_mute(bool mute) {
	tlv320_modify_reg(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void tlv320_set_speaker_volume(float left_dB) {
	tlv320_write_reg(AIC31XX_LANALOGSPL,tlv320_convert_analog_gain_to_register_value(left_dB) );
}

void tlv320_set_speaker_gain(float gaindb) {
	// Round the input to the nearest integer (to handle potential float inaccuracies)
	int roundedGain = gaindb + 0.5f;

	uint8_t gain = 0;
	switch (roundedGain) {
		case 12:
			gain = 0x1;
			break;
		case 18:
			gain = 0x2;
			break;
		case 24:
			gain = 0x3;
			break;
		default:
			gain = 0x0;
	}
	tlv320_modify_reg(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_GAIN_MASK, gain);
}

void tlv320_set_mic_pga_enable(bool enable) {
	tlv320_modify_reg(AIC31XX_MICPGA, AIC31XX_MICPGA_ENABLE_MASK, enable ? 0x0 : 1);
}

void tlv320_set_mic_pga_gain(float gain) {
	tlv320_modify_reg(AIC31XX_MICPGA, AIC31XX_MICPGA_GAIN_MASK, tlv320_convert_mic_pga_gain_to_register_value(gain));
}

void tlv320_set_adc_gain(float adcGain) {
	tlv320_write_reg(AIC31XX_ADCVOL,tlv320_convert_adc_volume_to_register_value(adcGain) );
}

esp_err_t tlv320_init(audio_hal_codec_config_t *cfg) {
    ESP_ERROR_CHECK(i2c_master_init());
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
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    tlv320_write_reg(AIC31XX_RESET, 0x1);
    vTaskDelay(pdMS_TO_TICKS(10));

    tlv320_modify_reg(AIC31XX_IFACE1, AIC31XX_IFACE1_DATATYPE_MASK, AIC31XX_I2S_MODE);
    tlv320_set_word_length(AIC31XX_WORD_LEN_16BITS);

    tlv320_set_clk_mux(AIC31XX_PLL_CLKIN_BCLK, AIC31XX_CODEC_CLKIN_PLL);
    tlv320_set_pll(1, 2, 48, 0);
    tlv320_set_pll_power(true);
    vTaskDelay(pdMS_TO_TICKS(15));

    tlv320_set_ndac_val(6);
    tlv320_set_ndac_power(true);
    tlv320_set_mdac_val(4);
    tlv320_set_mdac_power(true);
    tlv320_set_dosr_val(128);

    tlv320_enable_dac();
    tlv320_set_dac_mute(false);
    tlv320_set_dac_volume(0.0f, 0.0f);

    tlv320_enable_headphone_amp();
    tlv320_set_headphone_mute(false);
    tlv320_set_headphone_volume(-35.0f, -35.0f); // 0dB
    tlv320_set_headphone_gain(0.0f, 0.0f);
    tlv320_set_headphone_line_mode(true);

    // tlv320_enable_speaker_amp();
    // tlv320_set_speaker_mute(false);
    // tlv320_set_speaker_gain(0.0f);
    // tlv320_set_speaker_volume(0.0f);

    return ESP_OK;
}