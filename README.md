# FPGA Logic Analyzer

A parameterizable on-chip logic analyzer for the **iCEBreaker FPGA** (iCE40UP5K).
Captures internal FPGA signals into a circular buffer with configurable pre/post-trigger
capture, streams data over UART to a Python host tool, and exports to VCD/CSV for
analysis in GTKWave or a spreadsheet.

---

## Architecture

```
                         ┌──────────────────────────────────────────────────────┐
                         │                  iCEBreaker FPGA                      │
                         │                                                        │
   UART ─────────────────►  uart_rx                               uart_tx  ◄─────┤── UART
  (host)                 │     │                                     ▲            │  (host)
                         │     ▼                                     │            │
                         │  uart_cmd_parser ──────────────────────► │            │
                         │     │  ▲                                  │            │
                         │     │  │ status/readout                   │            │
                         │     ▼  │                                  │            │
                         │  config_regs                   rle_encoder             │
                         │     │                               ▲                  │
                         │     │ trig_mode/mask/value/depths   │                  │
                         │     ▼                               │ samples          │
                         │  trigger_engine ◄──────────┐        │                  │
                         │     │ trigger_fire          │        │                  │
                         │     ▼                       │        │                  │
                         │  capture_controller_fsm ────┼────────┘                 │
                         │     │ wr_addr/data/en  prev_data                        │
                         │     ▼                                                   │
                         │  circular_capture_buffer (BRAM)                        │
                         │                                                        │
                         │  probe_pins ──── IOB flop ──► probe_reg (to FSM)       │
                         └──────────────────────────────────────────────────────┘
```

### Data Flow

1. `probe_pins` are registered at the IOB (one flip-flop per pin) on every rising clock edge.
2. `trigger_engine` evaluates the trigger condition combinationally every cycle.
3. `capture_controller_fsm` manages the capture state machine:
   - **ARMED**: writes probe data into the circular BRAM continuously
   - **TRIGGERED**: latches the trigger address and computes linearisation parameters
   - **POST_CAPTURE**: captures post-trigger samples
   - **COMPLETE**: holds data and awaits host readout
   - **READOUT**: streams samples to the command parser
4. `uart_cmd_parser` receives host commands, controls the FSM, and sends responses
5. `rle_encoder` optionally compresses the sample stream before UART transmission

---

## Module Descriptions

| Module | Description |
|--------|-------------|
| `uart_rx` | 8N1 UART receiver with 2-FF synchroniser; outputs `rx_data`/`rx_valid` pulses |
| `uart_tx` | 8N1 UART transmitter; `tx_ready`/`tx_valid` handshake |
| `uart_cmd_parser` | Framed command protocol handler; two concurrent FSMs (RX parser + TX responder) |
| `trigger_engine` | Pure combinational trigger: IMMEDIATE, EQUALITY, RISING, FALLING |
| `circular_capture_buffer` | Thin BRAM wrapper (yosys infers iCE40 EBRs); no address logic |
| `capture_controller_fsm` | Core acquisition FSM; circular addressing, pre/post split, linearisation |
| `rle_encoder` | Change-only run-length encoding; passthrough mode for raw transfer |
| `config_regs` | Write-only register bank with combinational read; 7 registers |
| `top_logic_analyzer` | Top-level integration; wires all modules, drives LEDs |

---

## UART Command Protocol

### Packet Frame

```
Byte 0:   0xAA         Magic / start-of-frame
Byte 1:   CMD          Command code
Byte 2:   LEN          Payload byte count N (0–8)
Byte 3…:  PAYLOAD      N payload bytes
Byte 3+N: CHK          XOR of bytes 1 through 3+N-1
```

**Checksum**: `chk = CMD ^ LEN ^ payload[0] ^ ... ^ payload[N-1]`

### Command Table

| CMD  | Name       | Payload (H→D)                       | Response Payload (D→H)                         |
|------|------------|-------------------------------------|------------------------------------------------|
| 0x01 | PING       | —                                   | 0x55                                           |
| 0x02 | SET_REG    | reg_addr, value[31:0] BE (5 bytes)  | 0x00=OK / error code                           |
| 0x03 | GET_REG    | reg_addr (1 byte)                   | value[31:0] BE (4 bytes)                       |
| 0x04 | ARM        | —                                   | 0x00=OK / 0x01=already armed                   |
| 0x05 | ABORT      | —                                   | 0x00=OK                                        |
| 0x06 | STATUS     | —                                   | state, trig_pos[15:8], trig_pos[7:0], rle_en   |
| 0x07 | READ_DATA  | —                                   | raw: `0xAA 0x07 LEN_HI LEN_LO [bytes] CHK`    |
|      |            |                                     | rle: `0xAA 0x07 0xFF 0xFF [bytes] 0xAA 0x87`  |

### Analyzer State Codes

| Code | State        |
|------|--------------|
| 0x00 | IDLE         |
| 0x01 | ARMED        |
| 0x02 | TRIGGERED    |
| 0x03 | POST_CAPTURE |
| 0x04 | COMPLETE     |
| 0x05 | READOUT      |

### Register Map

| Addr | Name            | Reset  | Description                     |
|------|-----------------|--------|---------------------------------|
| 0x00 | TRIG_MODE       | 0x00   | 0=IMM, 1=EQ, 2=RISE, 3=FALL    |
| 0x01 | TRIG_MASK       | 0xFFFF | Trigger channel mask            |
| 0x02 | TRIG_VALUE      | 0x0000 | Trigger equality reference      |
| 0x03 | PRE_TRIG_DEPTH  | 256    | Pre-trigger sample count        |
| 0x04 | POST_TRIG_DEPTH | 768    | Post-trigger sample count       |
| 0x05 | RLE_EN          | 0x00   | 0=raw, 1=RLE transfer           |
| 0x06 | PROBE_MASK      | 0xFFFF | Active probe channels (hint)    |

---

## Resource Usage (iCE40UP5K)

| Resource | Used | Budget | Utilisation |
|----------|------|--------|-------------|
| LUTs     | ~520 | 5280   | ~10%        |
| FFs      | ~385 | 5280   | ~7%         |
| EBRs     | 4    | 30     | 13%         |
| DSPs     | 0    | 8      | 0%          |

The analyzer uses only ~10% of the FPGA's LUTs, leaving ~90% for user logic
being probed — the correct resource profile for an embedded instrument core.

---

## Build Steps

### Prerequisites

- [yosys](https://yosyshq.net/yosys/) (synthesis)
- [nextpnr-ice40](https://github.com/YosysHQ/nextpnr) (place-and-route)
- [icepack / iceprog](https://github.com/YosysHQ/icestorm) (bitstream tools)
- [iverilog](http://iverilog.icarus.com/) (simulation)
- Python 3.10+ with `pyserial` (`pip install pyserial`)

### Simulation

```bash
# Run all simulation tests

make sim-all

# Run individual module tests
make sim-uart        # UART RX/TX/loopback
make sim-trigger     # trigger_engine
make sim-buf         # circular_capture_buffer
make sim-fsm         # capture_controller_fsm
make sim-rle         # rle_encoder
make sim-top         # end-to-end integration

# Open waveforms
make wave-top        # requires gtkwave
```

### Synthesis and Programming

```bash
# Build bitstream
make synth

# Program iCEBreaker (connect via USB before running)
make prog
```

### Hardware Validation (Real Board)

```bash
# Full hardware validation suite (synth + prog + protocol + raw/rle capture checks)
python -m host.hw_validate --port /dev/ttyUSB1 --deep-diagnostics

# Equivalent make target
make hw-validate HW_PORT=/dev/ttyUSB1
```

### UART Pinpoint Diagnostics

Use these when hardware validation fails and you need to isolate the UART path:

```bash
# Program a minimal UART diagnostic image (beacon + byte echo)
make uart-diag-prog HW_PORT=/dev/ttyUSB1

# Run low-level UART diagnostics against that image
make uart-pinpoint HW_PORT=/dev/ttyUSB1
```

---

## Host Tool Usage

```bash
# Ping device
python -m host.la_cli status --port /dev/ttyUSB0

# Capture with IMMEDIATE trigger, save to VCD
python -m host.la_cli capture --port /dev/ttyUSB0 --out capture.vcd

# Rising-edge trigger on bit 0, display in terminal
python -m host.la_cli capture --port /dev/ttyUSB0 \
    --trig-mode RISING --trig-mask 0x0001 \
    --pre 256 --post 512 \
    --probe-names CLK MOSI MISO CS IRQ \
    --display

# Equality trigger: capture when bits [7:4] = 0b1010
python -m host.la_cli capture --port /dev/ttyUSB0 \
    --trig-mode EQUALITY --trig-mask 0x00F0 --trig-value 0x00A0 \
    --out capture.csv

# View a saved CSV file in the terminal
python -m host.la_cli show capture.csv --probe-names CLK MOSI MISO CS

# Abort/reset device
python -m host.la_cli reset --port /dev/ttyUSB0
```

---

## Example Capture Session

```
$ python -m host.la_cli capture --port /dev/ttyUSB0 \
    --trig-mode RISING --trig-mask 0x0001 \
    --pre 128 --post 128 --display \
    --probe-names CLK MOSI MISO CS

Connecting to /dev/ttyUSB0 @ 115200 baud...
Device OK
Configuring: trig=RISING  mask=0x0001  value=0x0000  pre=128  post=128  rle=off
Arming...
Capture complete: 257 samples  trigger at [128]

  Logic Analyzer Capture  [257 samples @ 12 MHz | pre=10.7µs  post=10.7µs | 14:23:55]

     CLK │▄▄▄▀▀▀▄▄▄▀▀▀▄▄▄▀▀▀▄▄▄▀▀▀▄▄▄▀▀▀▄▄▄▀▀▀▄▄▄│
    MOSI │▄▄▄▄▄▄▄▄▄▄▄▀▀▀▀▀▀▀▀▀▀▀▀▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄│
    MISO │▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▄▄▄▄▄▄▄▄▄▄▄▄▄▀▀▀▀▀▀▀▀▀▀▀│
      CS │▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀│
         │                     ^                  │
                               T=0

  Samples[0..15]: 0001 0001 0000 0000 0001 0001 0000 0000 ...
  Trigger sample [128]: 0x0001

VCD written to capture.vcd  (257 samples, trigger at sample 128)
```

Open `capture.vcd` in GTKWave:
```bash
gtkwave capture.vcd
```

---

## Probe Hookup

To probe internal FPGA signals in your own design, add `top_logic_analyzer`
as a submodule and connect your signals to `probe_pins`:

```systemverilog
// In your top-level design:
logic [15:0] my_probe_bus;
assign my_probe_bus = {
    8'h00,           // reserved
    spi_cs_n,        // bit 7
    spi_clk,         // bit 6
    spi_mosi,        // bit 5
    spi_miso,        // bit 4
    uart_rx,         // bit 3
    uart_tx,         // bit 2
    data_valid,      // bit 1
    busy             // bit 0
};

top_logic_analyzer #(
    .PROBE_WIDTH (16),
    .BUFFER_DEPTH(1024)
) u_la (
    .clk         (clk_12mhz),
    .rst_n       (rst_n),
    .uart_rx_pin (la_uart_rx),
    .uart_tx_pin (la_uart_tx),
    .probe_pins  (my_probe_bus),
    .led_armed   (la_led_armed),
    .led_triggered(la_led_trig),
    .led_complete(la_led_done)
);
```

---

## RLE Compression

Run-length encoding reduces transfer time for captures with many repeated values
(common for slow bus protocols, idle periods, etc.).

**Token format:**
- Literal `(3 bytes)`: `0x00  hi  lo`  — new sample value
- Run `(1 byte)`:     `0x80 | count`   — previous value repeats *count* times (1–127)

**Example**: 200 identical samples compress from 400 bytes → 8 bytes (~98% reduction).
**Worst case**: every sample unique → 3 bytes each vs 2 raw (1.5× expansion).

Enable with `--rle` flag or by writing `REG_RLE_EN=1`.

---

## Limitations and Future Improvements

| Limitation | Future Improvement |
|---|---|
| Single clock domain (12 MHz) | PLL for configurable sample rates up to 48 MHz |
| 16-bit probe bus | Configurable width; 32-bit bus using 8 EBRs |
| 1024-sample buffer | Deeper buffer using SPRAM (128 KB available on UP5K) |
| No external trigger input | PMOD-based external trigger with edge detection |
| UART-only interface | SPI or USB-CDC for higher bandwidth |
| No multi-channel trigger | Compound trigger expressions |
| No streaming capture | Continuous streaming mode (no pre-trigger, unlimited depth) |

---

## Resume Description

> Designed and implemented a parameterizable on-chip logic analyzer for the
> iCEBreaker FPGA (Lattice iCE40UP5K) in synthesizable SystemVerilog.
> Features include a 16-bit probe bus, configurable masked equality and
> edge-based triggers, circular pre-trigger buffering (1K samples / 4 EBRs),
> a framed UART command protocol with XOR checksum, optional RLE compression,
> and a Python host tool with GTKWave VCD export and terminal waveform display.
> Verified with self-checking per-module testbenches covering 15+ corner cases
> including buffer wrap, partial pre-trigger, and arm/disarm cycling.
> Resource utilisation: ~520 LUTs (~10% of budget), leaving 90% for user logic.

---

## File Structure

```
fpga-logic-analyzer/
├── rtl/
│   ├── uart_rx.sv              UART receiver (8N1, 2-FF synchroniser)
│   ├── uart_tx.sv              UART transmitter
│   ├── uart_cmd_parser.sv      Command protocol FSM + TX responder
│   ├── trigger_engine.sv       Pure combinational trigger evaluation
│   ├── circular_capture_buffer.sv  BRAM wrapper (yosys EBR inference)
│   ├── capture_controller_fsm.sv   Core acquisition state machine
│   ├── rle_encoder.sv          RLE / passthrough sample encoder
│   ├── config_regs.sv          Configuration register bank
│   └── top_logic_analyzer.sv   Top-level integration
├── tb/
│   ├── tb_uart_rx.sv           UART RX testbench
│   ├── tb_uart_tx.sv           UART TX testbench
│   ├── tb_uart_loopback.sv     TX→RX loopback (all 256 values)
│   ├── tb_trigger_engine.sv    Trigger testbench (all modes, edge cases)
│   ├── tb_circular_capture_buffer.sv  BRAM testbench
│   ├── tb_capture_controller_fsm.sv   FSM testbench (5 scenarios)
│   ├── tb_rle_encoder.sv       RLE encoder testbench
│   └── tb_top_logic_analyzer.sv  End-to-end integration testbench
├── host/
│   ├── la_protocol.py          Packet building, parsing, constants
│   ├── la_device.py            Serial port abstraction (pyserial)
│   ├── la_capture.py           Capture orchestration + data model
│   ├── la_export.py            VCD and CSV exporters
│   ├── la_display.py           Terminal ASCII waveform renderer
│   └── la_cli.py               argparse CLI entry point
├── constraints/
│   └── icebreaker.pcf          iCEBreaker pin assignments
├── Makefile                    sim-all / synth / prog targets
└── README.md                   This file
```
