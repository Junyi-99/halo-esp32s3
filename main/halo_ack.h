/* Strict Halo ACK detector. Plain C with no ESP dependencies.
 *
 * Input: HALO_ACK_SAMPLES raw 16 MS/s words of packed signed 10-bit IQ with the
 * LO on the 2405 MHz carrier. An ACK is accepted only if the demodulated bits
 * contain exactly: 32-bit address, 6-bit length 0, the expected 2-bit PID,
 * NO_ACK = 1 and a valid CRC16 over those 41 bits (57 bits in total).
 */
#pragma once
#include <stdbool.h>

#include "halo_synth.h"

#define HALO_ACK_SAMPLES    16380u
#define HALO_ACK_BITS       57u
#define HALO_ACK_BODY_BITS  41u                          /* address + length + PID + NO_ACK */
#define HALO_ACK_DECIMATION 8u                           /* 16 MS/s -> 2 MS/s before FM demod */
#define HALO_ACK_FM_RATE    (16000000.0f / HALO_ACK_DECIMATION)
#define HALO_ACK_SAMPLES_PER_BIT 16                      /* 2 MS/s / 125 kbit/s */
#define HALO_ACK_MAX_FM     (HALO_ACK_SAMPLES / HALO_ACK_DECIMATION + 1)
#define HALO_ACK_MAX_BITS   128u

static float halo_ack_fm[HALO_ACK_MAX_FM];

/* Build the 57-bit pattern an ACK for (address, pid) must contain; returns its CRC16. */
static uint16_t halo_ack_pattern(uint32_t address, unsigned pid, uint8_t bits[HALO_ACK_BITS])
{
    for (unsigned i = 0; i < 32; i++) {
        bits[i] = (address >> (31 - i)) & 1;
    }
    for (unsigned i = 32; i < 38; i++) {
        bits[i] = 0;                       /* payload length 0 */
    }
    bits[38] = (pid >> 1) & 1;
    bits[39] = pid & 1;
    bits[40] = 1;                          /* NO_ACK */
    uint16_t crc = halo_crc16(bits, HALO_ACK_BODY_BITS);
    for (unsigned i = 0; i < 16; i++) {
        bits[HALO_ACK_BODY_BITS + i] = (crc >> (15 - i)) & 1;
    }
    return crc;
}

/* Decimate by 8, remove DC, FM-demodulate into halo_ack_fm[] (Hz) and apply a
 * 3-tap moving average. Returns the number of FM samples.
 */
static unsigned halo_ack_demodulate(const uint32_t *raw, unsigned count)
{
    int64_t sum_i = 0, sum_q = 0;
    for (unsigned j = 0; j < count; j++) {
        sum_i += halo_sign10(raw[j]);
        sum_q += halo_sign10(raw[j] >> 10);
    }
    float dc_i = (float)sum_i / count;
    float dc_q = (float)sum_q / count;

    const float hz_per_radian = HALO_ACK_FM_RATE / (2.0f * HALO_PI);
    float previous_i = 0, previous_q = 0;
    unsigned nfm = 0;
    for (unsigned j = 0; j + HALO_ACK_DECIMATION <= count; j += HALO_ACK_DECIMATION) {
        int si = 0, sq = 0;
        for (unsigned k = 0; k < HALO_ACK_DECIMATION; k++) {
            si += halo_sign10(raw[j + k]);
            sq += halo_sign10(raw[j + k] >> 10);
        }
        float i = si - HALO_ACK_DECIMATION * dc_i;
        float q = -(sq - HALO_ACK_DECIMATION * dc_q);   /* conjugate: ESP IQ is spectrally inverted */
        if (j) {
            halo_ack_fm[nfm++] = atan2f(q * previous_i - i * previous_q, i * previous_i + q * previous_q) * hz_per_radian;
        }
        previous_i = i;
        previous_q = q;
    }

    float prev = halo_ack_fm[0];
    for (unsigned j = 1; j + 1 < nfm; j++) {
        float current = halo_ack_fm[j];
        halo_ack_fm[j] = (prev + current + halo_ack_fm[j + 1]) / 3;
        prev = current;
    }
    return nfm;
}

/* Search the capture for the expected ACK. On success, address_us is the start of
 * the ACK address field relative to capture start and crc_out its CRC16.
 */
static bool halo_ack_decode(const uint32_t *raw, unsigned count, uint32_t address, unsigned pid,
                            uint32_t *address_us, uint16_t *crc_out)
{
    if (count > HALO_ACK_SAMPLES || count < HALO_ACK_DECIMATION * HALO_ACK_BITS * HALO_ACK_SAMPLES_PER_BIT) {
        return false;
    }
    uint8_t expected[HALO_ACK_BITS];
    uint16_t crc = halo_ack_pattern(address, pid, expected);
    unsigned nfm = halo_ack_demodulate(raw, count);

    /* Try 32 half-sample bit phases and three slicing thresholds (Hz) to tolerate
     * carrier offset. A bit is accepted only if the samples 1.5 us either side of
     * its centre agree with it; otherwise it is marked 2 and can never match.
     */
    const float thresholds[] = {-60000, -15000, 30000};
    for (unsigned phase = 0; phase < 32; phase++) {
        float first = 4 + phase * .5f;
        for (unsigned threshold = 0; threshold < 3; threshold++) {
            float level = thresholds[threshold];
            uint8_t bits[HALO_ACK_MAX_BITS];
            unsigned n = 0;
            for (float t = first; t + 5 < nfm && n < HALO_ACK_MAX_BITS; t += HALO_ACK_SAMPLES_PER_BIT) {
                unsigned k = (unsigned)t;
                float value = halo_ack_fm[k] + (t - k) * (halo_ack_fm[k + 1] - halo_ack_fm[k]);
                bool bit = value > level;
                bool early = halo_ack_fm[k - 3] > level;
                bool late = halo_ack_fm[k + 3] > level;
                bits[n++] = (early == bit && late == bit) ? bit : 2;
            }
            for (unsigned start = 0; start + HALO_ACK_BITS <= n; start++) {
                unsigned k = 0;
                while (k < HALO_ACK_BITS && bits[start + k] == expected[k]) {
                    k++;
                }
                if (k == HALO_ACK_BITS) {
                    /* FM sample index -> microseconds at 2 MS/s, plus half the decimation window. */
                    *address_us = (uint32_t)((first + start * HALO_ACK_SAMPLES_PER_BIT) * .5f + .75f);
                    *crc_out = crc;
                    return true;
                }
            }
        }
    }
    return false;
}
