#!/usr/bin/env python3
"""Send one command to a BenQ ScreenBar Halo through the ESP32-S3 controller.
Brightness 1..100 maps to raw 4C..FE; zero means off, not calibrated luminance.
"""
import argparse
import json
from pathlib import Path
import time

import serial

ROOT = Path(__file__).resolve().parent
PIDFILE = ROOT / '.halo-next-pid.json'

MODES = {'off': 0, 'front': 1, 'back': 2, 'both': 3}
LED_OFF = (0, 0, 0)
LED_COLORS = {'off': (32, 32, 32), 'front': (0, 32, 0), 'back': (0, 0, 32), 'both': (32, 0, 32)}

DEFAULT_ADDRESS = '16224CB1'
DEFAULT_PORT = '/dev/ttyACM0'


class HaloClient:
    def __init__(self, port):
        self.p = serial.Serial(port=None, baudrate=2000000, timeout=.2, write_timeout=3, exclusive=True)
        self.p.dtr = self.p.rts = False
        self.p.port = port

    def __enter__(self):
        self.p.open()
        try:
            self._sync()
            self.p.timeout = 5
            if self.command('HALO?') != 'HALO 3 ACK MAX_ATTEMPTS':
                raise RuntimeError('Onboard-generation firmware required.')
            return self
        except BaseException:
            self.p.close()
            raise

    def _sync(self):
        """Release any previous owner and wait until the ESP echoes a fresh marker."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            marker = ('SYNC ' + str(time.monotonic_ns())).encode()
            self.p.write(b'\nRELEASE\n' + marker + b'\n')
            retry = time.monotonic() + 1
            while time.monotonic() < retry:
                if self.p.readline().strip() == marker:
                    return
        raise RuntimeError('Cannot synchronize; close viewer and other scripts.')

    def command(self, line):
        self.p.write((line + '\n').encode())
        return self.p.readline().decode(errors='replace').strip()

    def __exit__(self, *exc):
        try:
            self.p.write(b'RELEASE\n')
        finally:
            self.p.close()


def parse_args():
    class Formatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        def __init__(self, prog):
            super().__init__(prog, max_help_position=32, width=100)

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=Formatter)
    ap.add_argument('--address', metavar='HEX', default=DEFAULT_ADDRESS,
                    help='4-byte pairing address in on-air hex order')
    ap.add_argument('--mode', choices=MODES, default='front',
                    help='which lamp side to light')
    ap.add_argument('--brightness', metavar='0-100', type=int, choices=range(101), default=50,
                    help='lamp brightness, 0 is off')
    ap.add_argument('--gain', metavar='0-10', type=int, choices=range(11), default=8,
                    help='TX gain')
    ap.add_argument('--attempts', metavar='1-5', type=int, choices=range(1, 6), default=3,
                    help='max sends while waiting for ACK')
    ap.add_argument('--pid', metavar='0-3', type=int, choices=range(4), default=argparse.SUPPRESS,
                    help='override next PID; default is the value saved by the previous run')
    ap.add_argument('--led', choices=('on', 'off'), default=argparse.SUPPRESS,
                    help='onboard RGB LED: on = turn on, blink when the lamp ACKs; off = LED 0 0 0; '
                         'omit to leave it untouched')
    ap.add_argument('--port', metavar='DEV', default=DEFAULT_PORT,
                    help='ESP serial port')
    a = ap.parse_args()

    try:
        a.address = int(a.address.replace(':', '').replace(' ', ''), 16)
    except ValueError:
        ap.error('address must be hex')
    if not 0 <= a.address <= 0xffffffff:
        ap.error('address must fit 4 bytes')

    return a


def load_pid(override):
    if override is not None:
        return override
    if PIDFILE.exists():
        return int(json.loads(PIDFILE.read_text())['next_pid']) % 4
    return 0


def save_pid(pid):
    temp = PIDFILE.with_suffix('.tmp')
    temp.write_text(json.dumps({'next_pid': (pid + 1) % 4}) + '\n')
    temp.replace(PIDFILE)


def set_led(client, rgb):
    cmd = 'LED ' + ' '.join(map(str, rgb))
    if client.command(cmd) != 'OK ' + cmd:
        raise RuntimeError('LED command failed')


def blink_led(client, rgb):
    set_led(client, LED_OFF)
    time.sleep(0.2)
    set_led(client, rgb)


def main():
    a = parse_args()
    pid = load_pid(getattr(a, 'pid', None))
    led = getattr(a, 'led', None)
    rgb = LED_COLORS[a.mode if a.brightness else 'off']

    with HaloClient(a.port) as client:
        if led is not None and client.command('LED?') != 'LED 48 RGB WS2812':
            raise RuntimeError('LED support missing')
        if led == 'on':
            set_led(client, rgb)
        elif led == 'off':
            set_led(client, LED_OFF)

        line = f'HALO {a.address:08X} {MODES[a.mode]} {a.brightness} {pid} {a.gain} {a.attempts}'
        response = client.command(line)
        f = response.split()
        if len(f) != 10 or f[0] != 'HALODONE' or f[1] not in ('ACK', 'NO_ACK', 'RX_ERROR'):
            raise RuntimeError('Unexpected reply: ' + response)

        save_pid(pid)
        print(response, flush=True)
        if f[1] != 'ACK':
            raise SystemExit('No verified ACK; lamp state is unknown.')

        if led == 'on':
            blink_led(client, rgb)


if __name__ == '__main__':
    main()
