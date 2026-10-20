/* S3-only extraction from ESPARGOS/esp-sdr f86da81 calibration/tuning
 * and cyber-murmel/esp-sdr-zephyr v0.1.3 DAC bank playback. See docs/NOTICE.
 *
 * Register addresses and bit positions below are undocumented ESP32-S3 PHY/modem
 * registers taken from those projects. They are valid for the pinned ESP-IDF only.
 */
#include <stdatomic.h>
#include <sys/lock.h>

#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_private/phy.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"

#include "halo_ack.h"
#include "radio.h"

_Static_assert(HALO_ACK_SAMPLES == HALO_RX_CAPTURE_WORDS, "ACK capture must fill exactly one bank");

/* Keep the heap allocator away from the three modem SRAM banks. */
SOC_RESERVE_MEMORY_REGION(RING_BANK_BASE, RING_BANK_BASE + 3u * RING_BANK_SIZE, halo_dump);

/* Modem registers (esp-sdr names where known). */
#define HALO_DAC_CTRL_REG      0x60033d64u   /* DAC playback: bit31 enable, bit19 start, low bits = length */
#define HALO_ADC_CTRL_REG      0x60033d5cu   /* ADC capture: bit31 enable, bit19 start, bit18 done, bit16 mode */
#define HALO_ADC_CONFIG_REG    0x60033d90u
#define HALO_ADC_CONFIG_VALUE  0x000c2040u
#define HALO_RX_CAL_REG        0x6001c02cu
#define HALO_RX_CAL_BYPASS_BIT (1u << 23)
#define HALO_PHY_PARAM_RX_OFFSET 288          /* byte offset into phy_param[]; bit 9 cleared before gain table */

#define HALO_CTRL_ENABLE  0x80000000u
#define HALO_CTRL_START   (1u << 19)
#define HALO_CTRL_DONE    (1u << 18)
#define HALO_ADC_MODE_RAW (1u << 16)

#define HALO_CHANNEL_MHZ   2412   /* Wi-Fi channel 1 centre, used only for the channel index */
#define HALO_CARRIER_MHZ   2405   /* actual Halo carrier set via RF frequency offset */
#define HALO_RX_GAIN_INDEX 40
#define HALO_SETTLE_US     3000

#define CPU_CYCLES_PER_US        240u                 /* 240 MHz CPU */
#define DAC_CYCLES_PER_SAMPLE    6u                   /* 240 MHz / 40 MS/s */
#define ADC_TIMEOUT_CYCLES       (2000u * CPU_CYCLES_PER_US)   /* 2 ms for a 1 ms capture */
#define RETRY_DELAY_MS           100

/* Private PHY symbols from libphy.a / ROM. Signatures follow esp-sdr. */
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void stop_tx_tone(unsigned);
extern void force_tx_gain(unsigned, unsigned, int);
extern void force_txon_mode(unsigned, unsigned, unsigned);
extern void force_rx_gain(unsigned, unsigned, unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void set_rf_freq_offset(unsigned, unsigned, int);
extern void set_rx_gain_table(unsigned, unsigned);
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern unsigned char phy_param[];

/* RX DC calibration state. The PHY library recalibrates on its own schedule and
 * bumps dc_generation through the wrapped set_rx_gain_cal_dc; prepare_rx then
 * redoes our fixed-frequency calibration.
 */
static atomic_uint dc_generation;
static unsigned calibrated_generation;
static bool calibrated;
static bool measurement;   /* true while our own calibration runs inside the PHY lock */
static bool rx_ready;

static portMUX_TYPE halo_tx_mux = portMUX_INITIALIZER_UNLOCKED;

/* Force the synthesizer to the LO path esp-sdr uses (clear bit 4 of I2C block 0x65 reg 1). */
static void select_lo(void)
{
    unsigned old = rom_chip_i2c_readReg(0x65, 1, 0);
    if (old & 0x10u) {
        rom_chip_i2c_writeReg(0x65, 1, 0, old & ~0x10u);
    }
}

/* Linker wraps (see main/CMakeLists.txt): keep the PHY on our carrier whenever it
 * retunes during calibration, and record every DC calibration pass.
 */
extern void __real_chip_v7_set_chan_ana(unsigned);
void __wrap_chip_v7_set_chan_ana(unsigned channel)
{
    __real_chip_v7_set_chan_ana(channel);
    if (measurement) {
        set_rf_freq_offset(0, HALO_CARRIER_MHZ, 0);
        select_lo();
    }
}

extern void __real_set_rx_gain_cal_dc(uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t,
                                      uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t);
void __wrap_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e,
                               uintptr_t f, uintptr_t g, uintptr_t h, uintptr_t i, uintptr_t j)
{
    __real_set_rx_gain_cal_dc(a, b, c, d, e, f, g, h, i, j);
    atomic_fetch_add(&dc_generation, 1);
}

static void calibrate_rx(void)
{
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    measurement = true;
    REG_CLR_BIT(HALO_RX_CAL_REG, HALO_RX_CAL_BYPASS_BIT);
    set_rf_freq_offset(0, HALO_CARRIER_MHZ, 0);
    select_lo();
    *(uint32_t *)(phy_param + HALO_PHY_PARAM_RX_OFFSET) &= ~0x200u;
    set_rx_gain_table(HALO_CARRIER_MHZ, 0);
    measurement = false;
    calibrated_generation = atomic_load(&dc_generation);
    calibrated = true;
    _lock_release(&lock);
}

static void prepare_rx(void)
{
    bool stale = !calibrated || atomic_load(&dc_generation) != calibrated_generation;
    if (rx_ready && !stale) {
        return;
    }
    if (stale) {
        calibrate_rx();
    }
    select_lo();
    set_chanfreq(HALO_CHANNEL_MHZ, 0);
    set_rf_freq_offset(0, HALO_CARRIER_MHZ, 0);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    force_rx_gain(0, HALO_RX_GAIN_INDEX, 0);
    select_lo();
    esp_rom_delay_us(HALO_SETTLE_US);
    rx_ready = true;
}

/* Play bank 0 then bank 1 through the DAC. Returns the CPU cycle count at the end
 * of playback so the ACK capture can report its turnaround. Runs with interrupts
 * masked: the bank switch is timed by busy-waiting on the cycle counter.
 */
static uint32_t IRAM_ATTR halo_play_banks(void)
{
    uint32_t saved = REG_READ(SRAM_OWNER_REG);
    uint32_t ctrl = HALO_CTRL_ENABLE | HALO_BANK_WORDS;

    portENTER_CRITICAL(&halo_tx_mux);
    REG_WRITE(SRAM_OWNER_REG, (saved & ~15u) | 1u);   /* bank 0 to DAC */
    REG_WRITE(HALO_DAC_CTRL_REG, 0);
    REG_WRITE(HALO_DAC_CTRL_REG, ctrl);
    REG_WRITE(HALO_DAC_CTRL_REG, ctrl | HALO_CTRL_START);
    uint32_t t = esp_cpu_get_cycle_count();
    while ((uint32_t)(esp_cpu_get_cycle_count() - t) < HALO_STEP * DAC_CYCLES_PER_SAMPLE) {
    }
    REG_WRITE(SRAM_OWNER_REG, (saved & ~15u) | 2u);   /* bank 1 to DAC */
    while ((uint32_t)(esp_cpu_get_cycle_count() - t) < HALO_TX_SAMPLES * DAC_CYCLES_PER_SAMPLE) {
    }
    REG_WRITE(HALO_DAC_CTRL_REG, 0);
    uint32_t tx_end = esp_cpu_get_cycle_count();
    REG_WRITE(HALO_ADC_CTRL_REG, 0);
    REG_WRITE(SRAM_OWNER_REG, saved);
    portEXIT_CRITICAL(&halo_tx_mux);
    return tx_end;
}

/* Linearly interpolate the 2 MS/s compact IQ in bank 2 up to 40 MS/s (20x).
 * halo_sign10 comes from halo_synth.h via halo_ack.h.
 */
static uint32_t halo_interpolate(unsigned sample)
{
    if (sample >= HALO_TX_SAMPLES) {
        return 0;
    }
    const uint32_t *input = HALO_INPUT;
    unsigned j = sample / 20u;
    unsigned rem = sample % 20u;
    uint32_t a = input[j];
    uint32_t b = j + 1 < HALO_TX_SAMPLES / 20u ? input[j + 1] : 0;
    int ai = halo_sign10(a), aq = halo_sign10(a >> 10);
    int bi = halo_sign10(b), bq = halo_sign10(b >> 10);
    int i = ai + (bi - ai) * (int)rem / 20;
    int q = aq + (bq - aq) * (int)rem / 20;
    return ((uint32_t)i & 1023u) | (((uint32_t)q & 1023u) << 10);
}

/* Fill both DAC banks from bank 2. Must run before every transmission because
 * the ACK capture overwrites bank 0.
 */
static void halo_expand(void)
{
    uint32_t *first = HALO_DAC_BANK0;
    uint32_t *second = HALO_DAC_BANK1;
    for (unsigned s = 0; s < HALO_BANK_WORDS; s++) {
        first[s] = halo_interpolate(s);
    }
    for (unsigned s = HALO_STEP - 64u; s < HALO_STEP - 64u + HALO_BANK_WORDS; s++) {
        second[s % HALO_BANK_WORDS] = halo_interpolate(s);
    }
}

/* Switch to RX and capture HALO_ACK_SAMPLES raw IQ words into bank 0.
 * The lamp answers about 184 us after our packet ends, so this must start
 * immediately: no retune and no settling delay here.
 */
static bool halo_capture_ack(uint32_t tx_end, uint32_t *turnaround_us)
{
    force_txon_mode(0, 0, 0);
    rom_pbus_xpd_tx_off();
    rom_pbus_workmode();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    force_rx_gain(0, HALO_RX_GAIN_INDEX, 0);
    select_lo();

    uint32_t saved = REG_READ(SRAM_OWNER_REG);
    uint32_t ctrl = HALO_CTRL_ENABLE | HALO_ADC_MODE_RAW | HALO_ACK_SAMPLES;

    portENTER_CRITICAL(&halo_tx_mux);
    REG_WRITE(HALO_ADC_CTRL_REG, 0);
    REG_WRITE(HALO_ADC_CONFIG_REG, HALO_ADC_CONFIG_VALUE);
    REG_WRITE(SRAM_OWNER_REG, (saved & ~15u) | 1u);   /* bank 0 to ADC */
    REG_WRITE(HALO_ADC_CTRL_REG, ctrl);
    REG_WRITE(HALO_ADC_CTRL_REG, ctrl | HALO_CTRL_START);
    uint32_t t = esp_cpu_get_cycle_count();
    *turnaround_us = (t - tx_end) / CPU_CYCLES_PER_US;
    REG_WRITE(HALO_ADC_CTRL_REG, ctrl);
    while (!(REG_READ(HALO_ADC_CTRL_REG) & HALO_CTRL_DONE)
           && (uint32_t)(esp_cpu_get_cycle_count() - t) < ADC_TIMEOUT_CYCLES) {
    }
    bool done = (REG_READ(HALO_ADC_CTRL_REG) & HALO_CTRL_DONE) != 0;
    REG_WRITE(HALO_ADC_CTRL_REG, 0);
    REG_WRITE(SRAM_OWNER_REG, saved);
    portEXIT_CRITICAL(&halo_tx_mux);

    rx_ready = true;
    return done;
}

void halo_radio_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
}

void halo_radio_prepare(void)
{
    rx_ready = false;
    prepare_rx();
}

halo_result_t halo_radio_send(uint32_t address, unsigned pid, unsigned gain, unsigned max_attempts)
{
    /* Uncalibrated TX gain register values for host indices 0..10; index 8 is the tested setting. */
    static const unsigned gains[] = {0, 16, 32, 48, 49, 81, 82, 115, 87, 119, 107};
    halo_result_t result = {0};

    for (unsigned attempt = 0; attempt < max_attempts; attempt++) {
        halo_expand();
        rom_pbus_workmode();
        rom_pbus_xpd_rx_on(0);
        rom_pbus_xpd_tx_on(1);
        force_tx_gain(gains[gain], 0, 0);
        select_lo();
        force_txon_mode(1, 0, 0);
        esp_rom_delay_us(HALO_SETTLE_US);

        uint32_t tx_end = halo_play_banks();
        result.attempts++;

        if (!halo_capture_ack(tx_end, &result.turnaround_us)) {
            result.rx_error = true;
            break;
        }
        result.ack = halo_ack_decode(HALO_RX_CAPTURE, HALO_ACK_SAMPLES, address, pid,
                                     &result.ack_address_us, &result.ack_crc);
        if (result.ack) {
            break;
        }
        if (attempt + 1 < max_attempts) {
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }

    rx_ready = false;
    prepare_rx();
    return result;
}
