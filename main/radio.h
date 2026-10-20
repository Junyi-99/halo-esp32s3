#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Three reserved 64 KiB SRAM banks starting at RING_BANK_BASE (see sram_guard.ld):
 *   bank 0: first half of the 40 MS/s DAC waveform; reused as the ACK capture buffer
 *   bank 1: second half of the DAC waveform
 *   bank 2: compact 2 MS/s IQ produced by halo_synthesize, kept across retries
 * Each bank holds HALO_BANK_WORDS 32-bit words of packed signed 10-bit I (bits 9:0)
 * and Q (bits 19:10).
 */
#define RING_BANK_BASE   0x3fcb0000u
#define RING_BANK_SIZE   0x10000u
#define HALO_BANK_WORDS  16380u
#define HALO_DAC_BANK0   ((uint32_t *)RING_BANK_BASE)
#define HALO_DAC_BANK1   ((uint32_t *)(RING_BANK_BASE + RING_BANK_SIZE))
#define HALO_INPUT       ((uint32_t *)(RING_BANK_BASE + 2u * RING_BANK_SIZE))
#define HALO_RX_CAPTURE  ((const uint32_t *)RING_BANK_BASE)
#define HALO_RX_CAPTURE_WORDS HALO_BANK_WORDS   /* equals HALO_ACK_SAMPLES; checked in radio.c */

/* The DAC plays bank 0 and then bank 1 back to back. Bank 1 is filled starting
 * HALO_STEP samples into the waveform so the switch happens with margin.
 */
#define HALO_STEP        (HALO_BANK_WORDS - 64u - 2048u)
#define HALO_TX_SAMPLES  26720u   /* 668 us at 40 MS/s */

/* Modem SRAM bank ownership register shared by the DAC and ADC paths. */
#define SRAM_OWNER_REG   0x600c101cu

typedef struct {
    unsigned attempts;        /* transmissions actually made */
    bool ack;                 /* exact, CRC-valid ACK received */
    bool rx_error;            /* ADC capture did not complete */
    uint32_t turnaround_us;   /* end of DAC playback to start of capture */
    uint32_t ack_address_us;  /* ACK address field start, relative to capture start */
    uint16_t ack_crc;         /* CRC16 of the accepted ACK */
} halo_result_t;

void halo_radio_init(void);

void halo_radio_prepare(void);

halo_result_t halo_radio_send(uint32_t address, unsigned pid, unsigned gain, unsigned max_attempts);
