#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "periph_sdcard.h"
#include "i2s_stream.h"
#include "fatfs_stream.h"
#include "mp3_decoder.h"
#include "esp_peripherals.h"
#include "input_key_service.h"
#include "periph_button.h"
#include "board.h"
#include "periph_encoder.h"
#include "equalizer.h"

#include "sdcard_list.h"
#include "sdcard_scan.h"

#include "esp_lv_adapter.h"
#include "esp_lcd_panel_ssd1306.h"
#include "driver/i2c_master.h"

static const char *TAG = "S6TM_MAIN";

lv_obj_t *label;

audio_pipeline_handle_t pipeline;
audio_element_handle_t i2s_stream_writer, mp3_decoder, fatfs_stream_reader, equalizer;
playlist_operator_handle_t sdcard_list_handle = NULL;

int player_volume = 20;

typedef enum {
    MODE_VOLUME,
    MODE_EQ_ADJUST,
} encoder_mode_t;

static encoder_mode_t s_mode       = MODE_VOLUME;
static int            s_eq_band    = 0;
static int            s_eq_gain[10] = { 4, 3, 6, -13, -4, -8, -2, 6, 8, 9,
                                        4, 3, 6, -13, -4, -8, -2, 6, 8, 9};
static lv_obj_t      *s_vol_bar    = NULL;
static lv_obj_t      *s_eq_bars[10] = {NULL};
static lv_obj_t      *s_mode_label = NULL;
static lv_obj_t      *s_band_label = NULL;

typedef enum {
    UI_UPDATE_VOLUME,
    UI_UPDATE_EQ_BAND,
    UI_UPDATE_MODE,
} ui_update_type_t;

typedef struct {
    ui_update_type_t type;
    int volume;
    int band;
    int band_gain;
    encoder_mode_t mode;
} ui_update_t;

static QueueHandle_t s_ui_queue = NULL;

// EQ band center frequencies for display
static const char *eq_band_names[] = {
    "31","62","125","250","500","1k","2k","4k","8k","16k"
};

void update_title(char* msg) {
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lv_label_set_text(label, msg);
        esp_lv_adapter_unlock();
    }
}

static const char* url_to_title(const char *url) {
    if (url == NULL) return "";
    const char *title = strrchr(url, '/');
    title = title ? title + 1 : url;
    static char buf[64];
    strncpy(buf, title, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *dot = strrchr(buf, '.');
    if (dot) *dot = '\0';
    return buf;
}

static void ui_set_volume(int vol) {
    if (esp_lv_adapter_lock(-1) != ESP_OK) return;
    lv_bar_set_value(s_vol_bar, vol, LV_ANIM_OFF);
    esp_lv_adapter_unlock();
}

static void ui_set_eq_band(int band, int gain) {
    if (esp_lv_adapter_lock(-1) != ESP_OK) return;
    lv_bar_set_value(s_eq_bars[band], gain, LV_ANIM_OFF);
    esp_lv_adapter_unlock();
}

static void ui_set_mode(encoder_mode_t mode, int band) {
    if (esp_lv_adapter_lock(-1) != ESP_OK) return;
    switch (mode) {
        case MODE_VOLUME:
            lv_label_set_text(s_mode_label, "VOL");
            lv_label_set_text(s_band_label, "");
            lv_obj_remove_flag(label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_band_label, LV_OBJ_FLAG_HIDDEN);
            break;
        case MODE_EQ_ADJUST:
            lv_label_set_text(s_mode_label, "EQ=");
            lv_label_set_text_fmt(s_band_label, "%s %+ddB",
                                  eq_band_names[band], s_eq_gain[band]);
            lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_band_label, LV_OBJ_FLAG_HIDDEN);
            break;
    }
    esp_lv_adapter_unlock();
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx) {
    audio_board_handle_t board_handle = (audio_board_handle_t) ctx;
    ui_update_t upd = {0};
    // ESP_LOGE(TAG, "CB fired type=%d data=%d", evt->type, (int)evt->data);
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        // ESP_LOGI(TAG, "[ * ] input key id is %d", (int)evt->data);
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_PLAY:
                if (s_mode == MODE_VOLUME) {
                    ESP_LOGI(TAG, "[ * ] [Play] input key event");
                    audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                    switch (el_state) {
                        case AEL_STATE_INIT :
                            ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                            audio_pipeline_run(pipeline);
                            break;
                        case AEL_STATE_RUNNING :
                            ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                            audio_pipeline_pause(pipeline);
                            break;
                        case AEL_STATE_PAUSED :
                            ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                            audio_pipeline_resume(pipeline);
                            break;
                        default :
                            ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
                    }
                } else {
                    s_eq_band--;
                    if (s_eq_band < 0) s_eq_band = 9;
                    upd.type = UI_UPDATE_MODE;
                    upd.mode = s_mode;
                    upd.band = s_eq_band;
                    xQueueOverwrite(s_ui_queue, &upd);
                    break;
                }
                break;
            case INPUT_KEY_USER_ID_SET:
                if (s_mode == MODE_VOLUME) {
                    ESP_LOGI(TAG, "[ * ] [Set] input key event");
                    ESP_LOGI(TAG, "[ * ] Stopped, advancing to the next song");
                    char *url = NULL;
                    audio_pipeline_stop(pipeline);
                    audio_pipeline_wait_for_stop(pipeline);
                    audio_pipeline_terminate(pipeline);
                    sdcard_list_next(sdcard_list_handle, 1, &url);
                    ESP_LOGW(TAG, "URL: %s", url);
                    update_title(url_to_title(url));
                    audio_element_set_uri(fatfs_stream_reader, url);
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_pipeline_run(pipeline);
                } else {
                    s_eq_band++;
                    if (s_eq_band > 9) s_eq_band = 0;
                    upd.type = UI_UPDATE_MODE;
                    upd.mode = s_mode;
                    upd.band = s_eq_band;
                    xQueueOverwrite(s_ui_queue, &upd);
                    break;
                }
                break;
            case INPUT_KEY_USER_ID_MODE:
                switch (s_mode) {
                case MODE_VOLUME:
                    s_mode = MODE_EQ_ADJUST;
                    break;
                case MODE_EQ_ADJUST:
                    s_mode = MODE_VOLUME;
                    break;
                }
                upd.type = UI_UPDATE_MODE;
                upd.mode = s_mode;
                upd.band = s_eq_band;
                xQueueOverwrite(s_ui_queue, &upd);
            break;
        }
    }

    return ESP_OK;
}

void sdcard_url_save_cb(void *user_data, char *url) {
    playlist_operator_handle_t sdcard_handle = (playlist_operator_handle_t)user_data;
    esp_err_t ret = sdcard_list_save(sdcard_handle, url);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail to save sdcard url to sdcard playlist");
    }
}

static esp_err_t periph_event_cb(audio_event_iface_msg_t *event, void *context) {
    audio_board_handle_t board_handle = (audio_board_handle_t)context;
    ui_update_t upd = {0};
    if (event->source_type == PERIPH_ID_ENCODER) {
        int dir = (event->cmd == PERIPH_ENCODER_CW) ? 1 : -1;
        switch (s_mode) {
            case MODE_VOLUME:
                player_volume += dir * 2;
                if (player_volume > 100) player_volume = 100;
                if (player_volume < 0)   player_volume = 0;
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                upd.type   = UI_UPDATE_VOLUME;
                upd.volume = player_volume;
                xQueueOverwrite(s_ui_queue, &upd);
                break;

            case MODE_EQ_ADJUST:
                s_eq_gain[s_eq_band] += dir;
                if (s_eq_gain[s_eq_band] >  13) s_eq_gain[s_eq_band] =  13;
                if (s_eq_gain[s_eq_band] < -13) s_eq_gain[s_eq_band] = -13;
                upd.type = UI_UPDATE_EQ_BAND;
                upd.band = s_eq_band;
                upd.band_gain = s_eq_gain[s_eq_band];
                upd.mode = s_mode;
                xQueueOverwrite(s_ui_queue, &upd);
                break;
        }
    }
    return ESP_OK;
}

void init_ui(void) {
    if (esp_lv_adapter_lock(-1) != ESP_OK) return;

    lv_obj_t *scr = lv_scr_act();

    // mode label top left
    s_mode_label = lv_label_create(scr);
    lv_obj_align(s_mode_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text(s_mode_label, "VOL");

    label = lv_label_create(scr);
    lv_obj_set_width(label, 90);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label, "Lecture SD...");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 10, 0);

    s_vol_bar = lv_bar_create(scr);
    lv_obj_set_size(s_vol_bar, 4, 60);
    lv_bar_set_range(s_vol_bar, 0, 100);
    lv_bar_set_value(s_vol_bar, player_volume, LV_ANIM_OFF);
    lv_bar_set_mode(s_vol_bar, LV_BAR_MODE_NORMAL);

    // this is the key — tell LVGL to draw it vertically
    lv_obj_set_style_bg_color(s_vol_bar, lv_color_white(), LV_PART_MAIN);

    lv_obj_align(s_vol_bar, LV_ALIGN_RIGHT_MID, 0, 0);

    // EQ bars — 10 small vertical bars across bottom
    int bar_w = 8;
    int bar_h = 40;
    int spacing = 12;
    int start_x = 2;
    for (int i = 0; i < 10; i++) {
        s_eq_bars[i] = lv_bar_create(scr);
        lv_obj_set_size(s_eq_bars[i], bar_w, bar_h);
        lv_bar_set_range(s_eq_bars[i], -13, 13);
        lv_bar_set_value(s_eq_bars[i], 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_eq_bars[i], lv_color_white(), LV_PART_MAIN);
        lv_obj_align(s_eq_bars[i], LV_ALIGN_BOTTOM_LEFT,
                     start_x + i * spacing, -5);
    }

    // band label bottom right
    s_band_label = lv_label_create(scr);
    lv_obj_add_flag(s_band_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(s_band_label, 90);
    lv_label_set_long_mode(s_band_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(s_band_label, "Lecture SD...");
    lv_obj_align(s_band_label, LV_ALIGN_TOP_MID, 10, 0);

    esp_lv_adapter_unlock();
}

void init_lvgl(void) {
    // Step 0: Create your esp_lcd panel and (optionally) panel_io with esp_lcd APIs
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_get_bus_handle(I2C_NUM_0, &i2c_bus);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &panel_io));
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = 64
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    // Step 1: Initialize the adapter
    esp_lv_adapter_config_t cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_lv_adapter_init(&cfg));

    // Step 2: Register a display (choose macro by interface)
    esp_lv_adapter_display_config_t disp_cfg = ESP_LV_ADAPTER_DISPLAY_SPI_MONO_DEFAULT_CONFIG(
        panel,
        panel_io,
        128,
        64,
        ESP_LV_ADAPTER_ROTATE_0,
        ESP_LV_ADAPTER_MONO_LAYOUT_VTILED
    );
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    assert(disp != NULL);

    // Step 3: (Optional) Register input device(s)
    // Create touch handle using esp_lcd_touch API (implementation omitted here)
    // esp_lcd_touch_handle_t touch_handle = /* ... */;
    // esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, touch_handle);
    // lv_indev_t *touch = esp_lv_adapter_register_touch(&touch_cfg);
    // assert(touch != NULL);

    // Step 4: Start the adapter task
    ESP_ERROR_CHECK(esp_lv_adapter_start());
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << 2 | 1ULL << 4,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    s_ui_queue = xQueueCreate(1, sizeof(ui_update_t));

    ESP_LOGI(TAG, "[ 3 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    ESP_LOGI(TAG, "[3.1] Initialize keys on board");
    audio_board_key_init(set);

    periph_encoder_cfg_t enc_cfg = {
        .gpio_a = 38,
        .gpio_b = 39,
        .step   = 2,
    };
    esp_periph_handle_t encoder = periph_encoder_init(&enc_cfg);
    esp_periph_start(set, encoder);

    ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    esp_periph_set_register_callback(set, periph_event_cb, (void *)board_handle);
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    init_lvgl();
    init_ui();

    ESP_LOGI(TAG, "[3.2] Set up a sdcard playlist and scan sdcard music save to it");
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    sdcard_list_create(&sdcard_list_handle);
    if (gpio_get_level(2) && !gpio_get_level(4)) {
        sdcard_scan(sdcard_url_save_cb, "/sdcard/jp", 0, (const char *[]) {"mp3"}, 1, sdcard_list_handle);
    } else if (gpio_get_level(2)) {
        sdcard_scan(sdcard_url_save_cb, "/sdcard/sine", 0, (const char *[]) {"mp3"}, 1, sdcard_list_handle);
    } else {
        sdcard_scan(sdcard_url_save_cb, "/sdcard", 0, (const char *[]) {"mp3"}, 1, sdcard_list_handle);
    }
    sdcard_list_show(sdcard_list_handle);

    ESP_LOGI(TAG, "[ 3 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create mp3 decoder to decode mp3 file and set custom read callback");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    equalizer_cfg_t eq_cfg = DEFAULT_EQUALIZER_CONFIG();
    eq_cfg.set_gain = s_eq_gain;
    equalizer = equalizer_init(&eq_cfg);

    ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[] Create fatfs stream to read data from sdcard");
    char *url = NULL;
    sdcard_list_current(sdcard_list_handle, &url);
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);
    audio_element_set_uri(fatfs_stream_reader, url);
    update_title(url_to_title(url));

    ESP_LOGI(TAG, "[2.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, equalizer, "equalizer");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[2.4] Link it together [sd]-->file-->mp3_decoder-->equalizer-->i2s_stream-->[codec_chip]");
    const char *link_tag[4] = {"file", "mp3", "equalizer", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 4);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[ 5.1 ] Start audio_pipeline");
    audio_hal_set_volume(board_handle->audio_hal, 20);
    ui_set_volume(20);
    gpio_set_level(21, 0);
    for (int i = 0; i < 10; i++) {
        ui_set_eq_band(i, s_eq_gain[i]);
    }

    while (1) {
        ui_update_t upd;
        if (xQueueReceive(s_ui_queue, &upd, 0) == pdTRUE) {
            switch (upd.type) {
                case UI_UPDATE_VOLUME:
                    ui_set_volume(upd.volume);
                    break;
                case UI_UPDATE_EQ_BAND: {
                    int gains[20];
                    for (int i = 0; i < 10; i++) {
                        gains[i] = s_eq_gain[i];
                        gains[i + 10] = s_eq_gain[i];
                        // lv_obj_set_size(s_eq_bars[i], 9, 40);
                    }
                    // lv_obj_set_size(s_eq_bars[upd.band], 10, 40);
                    ESP_LOGI(TAG, "setting band: %d to %d", upd.band, upd.band_gain);
                    equalizer_set_gain_info(equalizer, upd.band, upd.band_gain, true);
                    ui_set_eq_band(upd.band, upd.band_gain);
                    ui_set_mode(upd.mode, upd.band);
                    break;
                }
                case UI_UPDATE_MODE:
                    ui_set_mode(upd.mode, upd.band);
                        if (esp_lv_adapter_lock(-1) != ESP_OK) break;
                        int bar_w = 8;
                        int bar_h = 40;
                        int spacing = 12;
                        int start_x = 2;
                        for (int i = 0; i < 10; i++) {
                            lv_obj_set_size(s_eq_bars[i], bar_w, bar_h);
                            lv_obj_align(s_eq_bars[i], LV_ALIGN_BOTTOM_LEFT,
                                        start_x + i * spacing, -5);
                        }
                    if (s_mode == MODE_EQ_ADJUST) {
                        lv_obj_set_size(s_eq_bars[s_eq_band], 10, 40);
                        lv_obj_align(s_eq_bars[s_eq_band], LV_ALIGN_BOTTOM_LEFT,
                                    start_x + s_eq_band * spacing - 1, -5);
                    }
                    esp_lv_adapter_unlock();
                    break;
            }
        }

        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg,  pdMS_TO_TICKS(10));
        // ESP_LOGE(TAG, "EVENT source_type=%d cmd=%d data=%d", 
        //      msg.source_type, msg.cmd, (int)msg.data);
        if (ret != ESP_OK) {
            // ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT) {
            if (msg.source == (void *) mp3_decoder
                && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
                audio_element_info_t music_info = {0};
                audio_element_getinfo(mp3_decoder, &music_info);
                ESP_LOGI(TAG, "[ * ] Received music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                         music_info.sample_rates, music_info.bits, music_info.channels);
                audio_element_setinfo(i2s_stream_writer, &music_info);
                if (equalizer_set_info(equalizer, music_info.sample_rates, music_info.channels) != ESP_OK) {
                    break;
                }
                i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
                continue;
            }
            // Advance to the next song when previous finishes
            if (msg.source == (void *) i2s_stream_writer
                && msg.cmd == AEL_MSG_CMD_REPORT_STATUS) {
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                if (el_state == AEL_STATE_FINISHED) {
                    ESP_LOGI(TAG, "[ * ] Finished, advancing to the next song");
                    sdcard_list_next(sdcard_list_handle, 1, &url);
                    ESP_LOGW(TAG, "URL: %s", url);
                    audio_element_set_uri(fatfs_stream_reader, url);
                    update_title(url_to_title(url));
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                    audio_pipeline_run(pipeline);
                }
                continue;
            }
        }
    }

    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);
    audio_pipeline_unregister(pipeline, fatfs_stream_reader);
    audio_pipeline_unregister(pipeline, mp3_decoder);
    audio_pipeline_unregister(pipeline, equalizer);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    sdcard_list_destroy(sdcard_list_handle);
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(equalizer);
    audio_element_deinit(mp3_decoder);
    audio_element_deinit(fatfs_stream_reader);
}
