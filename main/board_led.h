#pragma once
#include <stdint.h>
#include "esp_err.h"

#define BOARD_LED_GPIO 48   /* single WS2812-compatible RGB LED, silk "IO48" */

esp_err_t board_led_set(uint8_t r, uint8_t g, uint8_t b);
