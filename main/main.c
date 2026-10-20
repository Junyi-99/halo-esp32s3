#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "board_led.h"
#include "halo_synth.h"
#include "radio.h"

#define USB_WRITE_TIMEOUT  pdMS_TO_TICKS(3000)
#define CMD_LINE_MAX           128

_Static_assert(HALO_COMPACT_WORDS * 20u == HALO_TX_SAMPLES, "compact IQ must expand to the DAC length");

/* Payload byte 0 (flags) of a control packet. */
#define FLAG_POWER      0x80
#define FLAG_FRONT      0x40
#define FLAG_BACK       0x01
#define FLAG_BRIGHTNESS 0x04   /* selector: this packet carries a brightness value */

/* Payload byte 1: raw brightness code. 1..100 % maps linearly onto 0x4C..0xFE. */
#define LEVEL_MIN 0x4c
#define LEVEL_MAX 0xfe

static const char HELP_TEXT[] =
    "BenQ ScreenBar Halo (1st generation) controller - ESP32-S3\n"
    "Firmware: halo-esp32s3 standalone v0.2\n"
    "Purpose: 2.4 GHz remote replacement + GPIO48 RGB LED control.\n"
    "Radio: 2405 MHz, 125 kbit/s GFSK. Not a Wi-Fi access point.\n"
    "Your paired lamp address: 16224CB1 (supply it in each HALO command).\n"
    "Commands (newline terminated); press Enter to show this help again.\n"
    "HELP | help | ?       Print this help; no RF transmission.\n"
    "HALO?                Query protocol capabilities.\n"
    "HALO ADDR MODE LEVEL PID GAIN ATTEMPTS\n"
    "  ADDR: 8 hexadecimal digits, on-air address byte order.\n"
    "  MODE: 0=off, 1=front, 2=back, 3=both.\n"
    "  LEVEL: 0..100; 0=off, 1..100 maps to raw 4C..FE.\n"
    "  PID: 0..3; advance modulo 4 for each new command.\n"
    "  GAIN: 0..10, uncalibrated RF index; tested setting 8.\n"
    "  ATTEMPTS: 1..5 maximum tries. Stop on matching CRC-valid ACK.\n"
    "  Example: HALO 16224CB1 2 50 0 8 3\n"
    "  This address is for your lamp; other lamps need their own paired address.\n"
    "  Reply: HALODONE status attempts payload crc16 synth_us total_us turnaround_us ack_address_us ack_crc\n"
    "LED R G B            GPIO48 RGB, each 0..255; 0 0 0=off.\n"
    "LED?                 Query LED pin/type.\n"
    "SYNC nonce           Echo synchronization marker.\n"
    "RELEASE              Acknowledge host cleanup.\n"
    "Each HALO command is bounded. No autonomous loop or Wi-Fi networking.\n"
    "READY\n";

static void reply(const char *text)
{
    usb_serial_jtag_write_bytes(text, strlen(text), USB_WRITE_TIMEOUT);
}

static void replyf(const char *format, ...)
{
    char line[192];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    reply(line);
}

static void handle_led(const char *line)
{
    unsigned r, g, b;
    char extra;
    if (sscanf(line, "LED %u %u %u %c", &r, &g, &b, &extra) != 3 || r > 255 || g > 255 || b > 255) {
        reply("ERR led_args\n");
        return;
    }
    esp_err_t e = board_led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
    if (e == ESP_OK) {
        replyf("OK LED %u %u %u\n", r, g, b);
    } else if (e == ESP_ERR_INVALID_STATE) {
        reply("ERR led_init\n");
    } else {
        reply("ERR led_tx\n");
    }
}

static void handle_halo(const char *args)
{
    unsigned addr, mode, level, pid, gain, max_attempts;
    char extra;
    int fields = sscanf(args, "%x %u %u %u %u %u %c", &addr, &mode, &level, &pid, &gain, &max_attempts, &extra);
    if (fields != 6 || mode > 3 || level > 100 || pid > 3 || gain > 10 || max_attempts < 1 || max_attempts > 5) {
        reply("ERR halo_args\n");
        return;
    }

    /* Mode 0 keeps the front-lamp bits of an ordinary command but with POWER clear. */
    uint8_t lamps = mode == 2 ? FLAG_BACK : mode == 3 ? (FLAG_FRONT | FLAG_BACK) : FLAG_FRONT;
    bool on = mode != 0 && level != 0;
    uint8_t flags = lamps | FLAG_BRIGHTNESS | (on ? FLAG_POWER : 0);
    uint8_t value = level ? (uint8_t)(LEVEL_MIN + ((level - 1) * (LEVEL_MAX - LEVEL_MIN) + 49) / 99) : LEVEL_MIN;

    int64_t start = esp_timer_get_time();
    halo_radio_prepare();
    int64_t synthesis_start = esp_timer_get_time();
    uint16_t crc = halo_synthesize(addr, flags, value, pid, HALO_INPUT);
    uint64_t synth_us = (uint64_t)(esp_timer_get_time() - synthesis_start);

    halo_result_t r = halo_radio_send(addr, pid, gain, max_attempts);
    const char *status = r.ack ? "ACK" : r.rx_error ? "RX_ERROR" : "NO_ACK";
    replyf("HALODONE %s %u %02X%02X %04X %" PRIu64 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %04X\n",
           status, r.attempts, (unsigned)flags, (unsigned)value, (unsigned)crc, synth_us,
           (uint64_t)(esp_timer_get_time() - start), r.turnaround_us, r.ack_address_us, (unsigned)r.ack_crc);
}

static void handle(const char *line)
{
    if (!strcmp(line, "HELP") || !strcmp(line, "help") || !strcmp(line, "?")) {
        reply(HELP_TEXT);
    } else if (!strncmp(line, "SYNC ", 5)) {
        reply(line);
        reply("\n");
    } else if (!strcmp(line, "RELEASE")) {
        reply("OK\n");
    } else if (!strcmp(line, "LED?")) {
        reply("LED 48 RGB WS2812\n");
    } else if (!strncmp(line, "LED ", 4)) {
        handle_led(line);
    } else if (!strcmp(line, "HALO?")) {
        reply("HALO 3 ACK MAX_ATTEMPTS\n");
    } else if (!strncmp(line, "HALO ", 5)) {
        handle_halo(line + 5);
    } else {
        reply("ERR command - type HELP or press Enter for instructions\n");
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 8192, .rx_buffer_size = 1024};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    halo_radio_init();
    reply(HELP_TEXT);

    char line[CMD_LINE_MAX];
    unsigned used = 0;
    bool overflow = false;
    for (;;) {
        char c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(20)) != 1) {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c != '\n') {
            if (used + 1 < sizeof(line)) {
                line[used++] = c;
            } else {
                overflow = true;
            }
            continue;
        }
        if (overflow) {
            reply("ERR line_too_long\n");
        } else if (used) {
            line[used] = 0;
            handle(line);
        } else {
            reply(HELP_TEXT);
        }
        used = 0;
        overflow = false;
    }
}
