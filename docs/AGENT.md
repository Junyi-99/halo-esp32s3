# Agent guide

Read this before changing the firmware. Lives in `docs/`; read it even if you only look for `AGENTS.md` at the root.

## Rules

- Nothing transmits unless the host sends a command. No boot-time TX, loops or retries beyond the requested attempts.
- ACK means exact address, empty payload, matching PID, NO_ACK=1, valid CRC16. Anything else is `NO_ACK`: lamp state unknown.
- Keep the `HELP` text complete, including the owner's address `16224CB1`. Help never transmits.
- ESP-IDF pinned to `25fe69f946311abdaf9ad56591f25fedbc20ac98`. Private PHY symbols and registers are version-specific; do not upgrade as cleanup.
- Do not flash, open the serial port or send any command to the device unless asked in that request. Build is the default check.
- Commit or push only when asked.

## Layout

- `main/main.c`: serial parser and replies.
- `main/halo_synth.h`: CRC16, frame, GFSK synthesis to 2 MS/s IQ.
- `main/halo_ack.h`: ACK demodulation and validation.
- `main/radio.c`: PHY setup, 20x interpolation, DAC playback, ACK capture, retries. All register addresses come from esp-sdr.
- `main/board_led.c`: GPIO48 WS2812 via RMT.
- `tools/halo_control.py`: host CLI.

Memory: three reserved 64 KiB SRAM banks at 0x3FCB0000 (`main/sram_guard.ld`). Banks 0/1 are the 40 MS/s DAC waveform, bank 2 the compact 2 MS/s IQ. ACK capture overwrites bank 0, so every attempt re-expands. The lamp answers ~184 µs after our packet ends; do not add delays before capture.

Owner's hardware: ESP32-S3-WROOM-1 N8R8 on `/dev/ttyACM0`, lamp address `16224CB1`, tested gain index 8.

## Build

```bash
. /home/junyi/esp/esp-idf/export.sh
cd /home/junyi/claude/screenbar-halo/halo-esp32s3
idf.py build
```

Expect zero warnings from `main/`. SDK and tools are already installed on this machine (`nuc`).

## Flash and check (only when asked)

```bash
idf.py -p /dev/ttyACM0 flash
python3 tools/halo_control.py --mode front --brightness 50
```

Expect `HALODONE ACK 1 ...`. If the port is busy, ask the user to close the other program.
