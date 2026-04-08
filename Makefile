# Makefile — FPGA Logic Analyzer
#
# Simulation (cocotb + iverilog):
#   make sim-uart       — UART RX / TX / loopback tests
#   make sim-trigger    — trigger_engine tests
#   make sim-buf        — circular_capture_buffer tests
#   make sim-fsm        — capture_controller_fsm tests
#   make sim-rle        — rle_encoder tests
#   make sim-top        — end-to-end top_logic_analyzer tests
#   make sim-all        — run all simulation tests
#
# Synthesis (requires yosys + nextpnr-ice40):
#   make synth          — synthesise for iCEBreaker
#   make prog           — program iCEBreaker via iceprog
#
# Misc:
#   make clean          — remove generated files
#   make hw-validate    — run end-to-end hardware validation script
#   make uart-diag-prog — program minimal UART diagnostic image
#   make uart-pinpoint  — run low-level UART path diagnostics

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
PYTHON    = python
YOSYS     = yosys
NEXTPNR   = nextpnr-ice40
ICEPACK   = icepack
ICEPROG   = iceprog

# ---------------------------------------------------------------------------
# cocotb / iverilog common settings
# ---------------------------------------------------------------------------
export SIM           = icarus
export COCOTB_REDUCED_LOG_FMT = 1

# RTL include path so all sources are found by iverilog
RTL_DIR = $(PWD)/rtl

# ---------------------------------------------------------------------------
# RTL source list (used by synthesis and some cocotb targets)
# ---------------------------------------------------------------------------
RTL_SRC = rtl/uart_rx.sv \
          rtl/uart_tx.sv \
          rtl/trigger_engine.sv \
          rtl/circular_capture_buffer.sv \
          rtl/config_regs.sv \
          rtl/capture_controller_fsm.sv \
          rtl/rle_encoder.sv \
          rtl/uart_cmd_parser.sv \
          rtl/top_logic_analyzer.sv

# ---------------------------------------------------------------------------
# Helper: run one cocotb test suite
#   $(call run_cocotb, TOPLEVEL, MODULE, VERILOG_SOURCES)
# ---------------------------------------------------------------------------
define run_cocotb
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=$(1) \
	  MODULE=$(2) \
	  VERILOG_SOURCES="$(3)"
endef

# ---------------------------------------------------------------------------
# Simulation targets
# ---------------------------------------------------------------------------
.PHONY: sim-uart sim-trigger sim-buf sim-fsm sim-rle sim-top sim-all

sim-uart:
	@echo "=== sim-uart: uart_rx ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=uart_rx MODULE=test_uart_rx \
	  VERILOG_SOURCES="$(RTL_DIR)/uart_rx.sv"
	@echo "=== sim-uart: uart_tx ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=uart_tx MODULE=test_uart_tx \
	  VERILOG_SOURCES="$(RTL_DIR)/uart_tx.sv"
	@echo "=== sim-uart: loopback ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=tb_loopback MODULE=test_uart_loopback \
	  VERILOG_SOURCES="$(PWD)/tb/tb_loopback.v $(RTL_DIR)/uart_rx.sv $(RTL_DIR)/uart_tx.sv"

sim-trigger:
	@echo "=== sim-trigger ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=trigger_engine MODULE=test_trigger_engine \
	  VERILOG_SOURCES="$(RTL_DIR)/trigger_engine.sv"

sim-buf:
	@echo "=== sim-buf ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=circular_capture_buffer MODULE=test_circular_capture_buffer \
	  VERILOG_SOURCES="$(RTL_DIR)/circular_capture_buffer.sv"

sim-fsm:
	@echo "=== sim-fsm ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=tb_capture_fsm MODULE=test_capture_controller_fsm \
	  VERILOG_SOURCES="$(PWD)/tb/tb_capture_fsm.v \
	                   $(RTL_DIR)/trigger_engine.sv \
	                   $(RTL_DIR)/circular_capture_buffer.sv \
	                   $(RTL_DIR)/capture_controller_fsm.sv"

sim-rle:
	@echo "=== sim-rle ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=rle_encoder MODULE=test_rle_encoder \
	  VERILOG_SOURCES="$(RTL_DIR)/rle_encoder.sv"

sim-top:
	@echo "=== sim-top ==="
	$(MAKE) -C tb -f $(PWD)/tb/Makefile.cocotb \
	  TOPLEVEL=top_logic_analyzer MODULE=test_top_logic_analyzer \
	  COMPILE_ARGS="-P top_logic_analyzer.CLK_FREQ=1000000 \
	                -P top_logic_analyzer.BAUD_RATE=100000 \
	                -P top_logic_analyzer.PROBE_WIDTH=8 \
	                -P top_logic_analyzer.BUFFER_DEPTH=64" \
	  VERILOG_SOURCES="$(RTL_DIR)/uart_rx.sv \
	                   $(RTL_DIR)/uart_tx.sv \
	                   $(RTL_DIR)/trigger_engine.sv \
	                   $(RTL_DIR)/circular_capture_buffer.sv \
	                   $(RTL_DIR)/config_regs.sv \
	                   $(RTL_DIR)/capture_controller_fsm.sv \
	                   $(RTL_DIR)/rle_encoder.sv \
	                   $(RTL_DIR)/uart_cmd_parser.sv \
	                   $(RTL_DIR)/top_logic_analyzer.sv"

sim-all: sim-uart sim-trigger sim-buf sim-fsm sim-rle sim-top
	@echo ""
	@echo "=== All simulation tests complete ==="

# ---------------------------------------------------------------------------
# Synthesis targets
# ---------------------------------------------------------------------------
.PHONY: synth prog hw-validate uart-diag-synth uart-diag-prog uart-pinpoint

DEVICE  = up5k
PACKAGE = sg48
TOP     = top_logic_analyzer
JSON    = build/synth.json
ASC     = build/top.asc
BIN     = build/top.bin

build/:
	mkdir -p build

$(JSON): $(RTL_SRC) | build/
	$(YOSYS) -p " \
	  read_verilog -sv $(RTL_SRC); \
	  synth_ice40 -top $(TOP) -json $(JSON)" \
	  2>&1 | tee build/yosys.log

$(ASC): $(JSON) constraints/icebreaker.pcf | build/
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) \
	  --pcf-allow-unconstrained \
	  --json $(JSON) --pcf constraints/icebreaker.pcf \
	  --asc $(ASC) \
	  2>&1 | tee build/nextpnr.log

$(BIN): $(ASC)
	$(ICEPACK) $(ASC) $(BIN)

synth: $(BIN)
	@echo "Bitstream: $(BIN)"

prog: $(BIN)
	$(ICEPROG) $(BIN)

HW_PORT ?= /dev/ttyUSB1
HW_BAUD ?= 115200

hw-validate:
	python3 -m host.hw_validate --port $(HW_PORT) --baud $(HW_BAUD) --deep-diagnostics --baud-sweep

UART_DIAG_TOP  = uart_diag_top
UART_DIAG_JSON = build/uart_diag.json
UART_DIAG_ASC  = build/uart_diag.asc
UART_DIAG_BIN  = build/uart_diag.bin

$(UART_DIAG_JSON): rtl/uart_rx.sv rtl/uart_tx.sv rtl/uart_diag_top.sv | build/
	$(YOSYS) -p " \
	  read_verilog -sv rtl/uart_rx.sv rtl/uart_tx.sv rtl/uart_diag_top.sv; \
	  synth_ice40 -top $(UART_DIAG_TOP) -json $(UART_DIAG_JSON)" \
	  2>&1 | tee build/yosys_uart_diag.log

$(UART_DIAG_ASC): $(UART_DIAG_JSON) constraints/icebreaker.pcf | build/
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) \
	  --pcf-allow-unconstrained \
	  --json $(UART_DIAG_JSON) --pcf constraints/icebreaker.pcf \
	  --asc $(UART_DIAG_ASC) \
	  2>&1 | tee build/nextpnr_uart_diag.log

$(UART_DIAG_BIN): $(UART_DIAG_ASC)
	$(ICEPACK) $(UART_DIAG_ASC) $(UART_DIAG_BIN)

uart-diag-synth: $(UART_DIAG_BIN)
	@echo "UART diagnostic bitstream: $(UART_DIAG_BIN)"

uart-diag-prog: $(UART_DIAG_BIN)
	$(ICEPROG) $(UART_DIAG_BIN)

uart-pinpoint:
	python3 -m host.uart_pinpoint --port $(HW_PORT) --baud $(HW_BAUD)

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
.PHONY: clean

clean:
	rm -rf tb/sim_build_* tb/__pycache__ tb/results.xml
	rm -f tb/*.vcd tb/*.fst tb/*.ghw
	rm -rf build/
