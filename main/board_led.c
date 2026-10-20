#include "board_led.h"

#include <stdbool.h>
#include <stddef.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

/* WS2812 timing at 10 MHz RMT resolution (0.1 us ticks):
 *   1 bit: 0.9 us high, 0.3 us low;  0 bit: 0.3 us high, 0.9 us low;  reset: >= 50 us low.
 */
#define LED_RMT_HZ        10000000
#define LED_T_LONG        9
#define LED_T_SHORT       3
#define LED_RESET_TICKS   2500
#define LED_BITS          24

static rmt_channel_handle_t led_channel;
static rmt_encoder_handle_t led_encoder;

static esp_err_t led_init(void)
{
    if (led_channel) {
        return ESP_OK;
    }
    rmt_tx_channel_config_t channel = {
        .gpio_num = BOARD_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    rmt_copy_encoder_config_t encoder = {};
    esp_err_t e = rmt_new_tx_channel(&channel, &led_channel);
    if (e == ESP_OK) {
        e = rmt_new_copy_encoder(&encoder, &led_encoder);
    }
    if (e == ESP_OK) {
        e = rmt_enable(led_channel);
    }
    if (e != ESP_OK) {
        if (led_encoder) {
            rmt_del_encoder(led_encoder);
        }
        if (led_channel) {
            rmt_del_channel(led_channel);
        }
        led_encoder = NULL;
        led_channel = NULL;
    }
    return e;
}

esp_err_t board_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_init() != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    /* WS2812 expects GRB, MSB first, followed by a reset gap. */
    rmt_symbol_word_t symbols[LED_BITS + 1];
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    for (unsigned j = 0; j < LED_BITS; j++) {
        bool one = (grb >> (LED_BITS - 1 - j)) & 1;
        symbols[j] = (rmt_symbol_word_t){
            .level0 = 1, .duration0 = one ? LED_T_LONG : LED_T_SHORT,
            .level1 = 0, .duration1 = one ? LED_T_SHORT : LED_T_LONG,
        };
    }
    symbols[LED_BITS] = (rmt_symbol_word_t){
        .level0 = 0, .duration0 = LED_RESET_TICKS,
        .level1 = 0, .duration1 = LED_RESET_TICKS,
    };

    rmt_transmit_config_t tx = {.loop_count = 0};
    esp_err_t e = rmt_transmit(led_channel, led_encoder, symbols, sizeof(symbols), &tx);
    if (e == ESP_OK) {
        e = rmt_tx_wait_all_done(led_channel, -1);
    }
    return e;
}
