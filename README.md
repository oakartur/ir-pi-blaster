# IR Pi Blaster

This repository contains a small utility for sending and receiving NEC/NECx
infrared frames using a Raspberry Pi and the [pigpio](https://abyz.me.uk/rpi/pigpio/)
GPIO library. It provides a single binary, `ir_nec_rpi`, that can decode signals
from a TSOP-style IR receiver and replay them on an IR LED driven directly from
one of the Pi's PWM-capable GPIO pins.

## Requirements

Install the runtime dependencies with pip:

```bash
python -m pip install -r requirements.txt
```

In addition you need the pigpio system library and build tooling when compiling
on Raspberry Pi OS:

```bash
sudo apt update
sudo apt install pigpio libpigpio-dev g++ make
```

The examples below assume you are running on a Raspberry Pi with
`pigpiod` stopped (the application links directly to the pigpio C library) and
you are executing the binary with `sudo` so pigpio can access the hardware.

## Building

Compile the receiver/transmitter with g++:

```bash
g++ -std=gnu++17 ir_nec_rpi.cpp -lpigpio -lpthread -O2 -o ir_nec_rpi
```

This produces the `ir_nec_rpi` executable in the project root. Ensure that
`libpigpio` is available on the system before running the binary.

## Usage

Run the program as root and choose the desired mode:

```bash
sudo ./ir_nec_rpi [OPTIONS]
```

Key options exposed by the CLI are:

| Option | Description |
| ------ | ----------- |
| `--mode=RX\|TX\|ALL` | Select the runtime mode. `RX` only listens for NEC/NECx packets, `TX` waits for button presses on the configured GPIO to send the payload, and `ALL` enables both simultaneously. |
| `--payload=0xFD020707` | Overrides the payload sent when transmitting. The value is a 32‑bit little-endian NEC/NECx frame. |
| `--leader=full\|half` | Chooses between a full (9 ms) or half (4.5 ms) leader mark when transmitting. |
| `--command <NAME>` | Sends the payload associated with `<NAME>` once and exits. The payload is looked up in the map file unless `--payload` is provided. |
| `--map <PATH>` | Location of the button map file used with `--command`. Defaults to `/home/aac/projects/ir-controller/button-map` with a fallback to the bundled `./button-map`. |
| `-h`, `--help` | Display the usage information. |

Example: transmit the `TOGGLE` command defined in `button-map` using the half
leader defaults:

```bash
sudo ./ir_nec_rpi --command TOGGLE --map ./button-map
```

To run the utility interactively with both RX and TX enabled, watching for a
hardware button on GPIO 17 to trigger transmissions:

```bash
sudo ./ir_nec_rpi --mode ALL
```

The application prints diagnostic information to standard output, including
payload decoding results in RX mode and timings for TX operations.

## Button map format

The `button-map` file stores symbolic names and payloads in the format
`NAME=0x<little-endian-hex>`. Comments can be added by prefixing a line with `#`.
You can provide your own map file via the `--map` option.
