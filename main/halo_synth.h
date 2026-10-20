/* Halo air protocol: CRC16, packed-IQ helper, packet encoder and GFSK synthesis.
 * Plain C with no ESP dependencies; halo_ack.h builds on it.
 *
 * Air interface: 125 kbit/s, deviation 160 kHz, Gaussian BT = 0.5. The phase is
 * integrated at 40 MS/s (320 samples per bit) and decimated by 20 to a compact
 * 2 MS/s IQ stream; radio.c interpolates it back to 40 MS/s for the DAC.
 *
 * Packet (81 bits): 8-bit preamble 0x55, 32-bit address, 6-bit length (=2),
 * 2-bit PID, NO_ACK bit (=0), 2 payload bytes, CRC16 over everything after the
 * preamble (polynomial 0x1021, init 0xFFFF).
 */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#define HALO_PACKET_BITS      81u
#define HALO_SAMPLES_PER_BIT  320                       /* 40 MS/s / 125 kbit/s */
#define HALO_DECIMATION       20                        /* 40 MS/s -> 2 MS/s */
#define HALO_LEAD_WORDS       20                        /* zero IQ before the preamble, 10 us */
#define HALO_COMPACT_WORDS    1336u                     /* bank 2 words written (HALO_TX_SAMPLES / 20) */
#define HALO_RADIUS           339                       /* Gaussian kernel half-width in samples */
#define HALO_PI               3.14159265358979323846f
#define HALO_PHASE_PER_SAMPLE (2.0f * HALO_PI * 160000.0f / 40000000.0f)

#include "halo_gaussian_table.h"   /* halo_kernel_cdf[2 * HALO_RADIUS + 2] */

static inline uint16_t halo_crc16(const uint8_t *bits, unsigned count)
{
    uint16_t crc = 0xffff;
    for (unsigned j = 0; j < count; j++) {
        unsigned mix = (crc >> 15) ^ bits[j];
        crc = (uint16_t)(crc << 1);
        if (mix) {
            crc ^= 0x1021;
        }
    }
    return crc;
}

/* Sign-extend the low 10 bits of a packed IQ word. Pass (word >> 10) for Q. */
static inline int halo_sign10(uint32_t v)
{
    int n = (int)(v & 1023u);
    return n >= 512 ? n - 1024 : n;
}

static inline void halo_append_bits(uint8_t *bits, unsigned *n, uint32_t value, unsigned width)
{
    for (unsigned i = width; i; i--) {
        bits[(*n)++] = (value >> (i - 1)) & 1;
    }
}

/* Write the 81-bit packet MSB first into bits[]; returns its CRC16. */
static inline uint16_t halo_packet(uint32_t address, uint8_t flags, uint8_t value, unsigned pid,
                            uint8_t bits[HALO_PACKET_BITS])
{
    unsigned n = 0;
    halo_append_bits(bits, &n, 0x55, 8);
    halo_append_bits(bits, &n, address, 32);
    halo_append_bits(bits, &n, 2, 6);       /* payload length */
    halo_append_bits(bits, &n, pid, 2);
    halo_append_bits(bits, &n, 0, 1);       /* NO_ACK = 0: we want an ACK */
    halo_append_bits(bits, &n, flags, 8);
    halo_append_bits(bits, &n, value, 8);
    uint16_t crc = halo_crc16(bits + 8, n - 8);
    halo_append_bits(bits, &n, crc, 16);
    return crc;
}

/* Gaussian-filtered NRZ value at 40 MS/s sample n, using the tabulated kernel CDF
 * so each constant-bit run contributes one subtraction instead of a convolution.
 */
static inline float halo_filtered_bit(const uint8_t *bits, int n)
{
    const int total = (int)HALO_PACKET_BITS * HALO_SAMPLES_PER_BIT;
    float filtered = 0;
    int t = n - HALO_RADIUS;
    int end = n + HALO_RADIUS;
    while (t <= end) {
        int bit, next;
        if (t < 0) {
            bit = 0;
            next = 0;
        } else if (t >= total) {
            bit = (int)HALO_PACKET_BITS - 1;
            next = end + 1;
        } else {
            bit = t / HALO_SAMPLES_PER_BIT;
            next = (bit + 1) * HALO_SAMPLES_PER_BIT;
        }
        if (next > end + 1) {
            next = end + 1;
        }
        float weight = halo_kernel_cdf[next - n + HALO_RADIUS] - halo_kernel_cdf[t - n + HALO_RADIUS];
        filtered += (bits[bit] ? 1.0f : -1.0f) * weight;
        t = next;
    }
    return filtered;
}

/* Synthesize the packet into out[HALO_COMPACT_WORDS] as packed signed 10-bit IQ
 * (I in bits 9:0, Q in bits 19:10, amplitude 128). Returns the packet CRC16.
 */
static inline uint16_t halo_synthesize(uint32_t address, uint8_t flags, uint8_t value, unsigned pid,
                                uint32_t *out)
{
    uint8_t bits[HALO_PACKET_BITS];
    uint16_t crc = halo_packet(address, flags, value, pid, bits);

    memset(out, 0, HALO_COMPACT_WORDS * sizeof(*out));
    float phase = 0;
    for (int n = 0; n < (int)HALO_PACKET_BITS * HALO_SAMPLES_PER_BIT; n++) {
        phase += halo_filtered_bit(bits, n) * HALO_PHASE_PER_SAMPLE;
        if (phase > HALO_PI) {
            phase -= 2 * HALO_PI;
        }
        if (phase < -HALO_PI) {
            phase += 2 * HALO_PI;
        }
        if (n % HALO_DECIMATION == 0) {
            int i = (int)lroundf(128 * cosf(phase));
            int q = (int)lroundf(128 * sinf(phase));
            out[HALO_LEAD_WORDS + n / HALO_DECIMATION] = ((uint32_t)i & 1023u) | (((uint32_t)q & 1023u) << 10);
        }
    }
    return crc;
}
