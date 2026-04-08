// top_logic_analyzer.sv — top-level integration for iCEBreaker board
//
// Instantiates and wires all sub-modules:
//   uart_rx → uart_cmd_parser → config_regs
//                             → capture_controller_fsm → circular_capture_buffer
//                             ↔ trigger_engine
//                             → rle_encoder → uart_cmd_parser → uart_tx
//
// Probe inputs are registered at the IOB level (one FF before entering
// the FSM) to ensure clean timing from PMOD pins.
//
// LEDs:
//   led_armed     — lit while ARMED or POST_CAPTURE
//   led_triggered — lit while POST_CAPTURE (trigger seen, capturing)
//   led_complete  — lit while COMPLETE or READOUT
//
// Target: iCEBreaker (iCE40UP5K), 12 MHz oscillator.
// Build: yosys + nextpnr-ice40 + icepack.
//
// Synthesis: iCE40-portable, no vendor IP.

`timescale 1ns/1ps
`default_nettype none

module top_logic_analyzer #(
    parameter int CLK_FREQ     = 12_000_000,
    parameter int BAUD_RATE    = 115_200,
    parameter int PROBE_WIDTH  = 16,
    parameter int BUFFER_DEPTH = 1024,
    parameter int ADDR_BITS    = $clog2(BUFFER_DEPTH)
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // UART (iCEBreaker FTDI USB-UART bridge)
    input  logic                    uart_rx_pin,
    output logic                    uart_tx_pin,

    // Probe inputs (16 PMOD pins)
    input  logic [PROBE_WIDTH-1:0]  probe_pins,

    // Status LEDs
    output logic                    led_armed,
    output logic                    led_triggered,
    output logic                    led_complete
);

    // Power-on reset to guarantee deterministic startup on hardware.
    logic [2:0] por_count = '0;
    logic       rst_n_sys = 1'b0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            por_count <= '0;
            rst_n_sys <= 1'b0;
        end else if (!rst_n_sys) begin
            por_count <= por_count + 3'd1;
            if (&por_count) rst_n_sys <= 1'b1;
        end
    end

    // -----------------------------------------------------------------------
    // Probe input register (IOB flop)
    // -----------------------------------------------------------------------
    logic [PROBE_WIDTH-1:0] probe_reg;
    always_ff @(posedge clk) probe_reg <= probe_pins;

    // -----------------------------------------------------------------------
    // UART RX
    // -----------------------------------------------------------------------
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       rx_valid_gated;
    logic       rx_error;  // framing error (logged via LED or ignored)
    localparam int UART_FRAME_CLKS = (CLK_FREQ / BAUD_RATE) * 12;
    localparam int RX_HOLDOFF_W    = $clog2(UART_FRAME_CLKS + 1);
    logic [RX_HOLDOFF_W-1:0] rx_holdoff_ctr = '0;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_rx (
        .clk     (clk),
        .rst_n   (rst_n_sys),
        .rx      (uart_rx_pin),
        .rx_data (rx_data),
        .rx_valid(rx_valid),
        .rx_error(rx_error)
    );

    // Ignore RX briefly after each TX byte launch to prevent parser self-reception
    // feedback loops on boards where UART TX can be observed on RX.
    always_ff @(posedge clk) begin
        if (!rst_n_sys) begin
            rx_holdoff_ctr <= '0;
        end else if (tx_valid) begin
            rx_holdoff_ctr <= RX_HOLDOFF_W'(UART_FRAME_CLKS);
        end else if (rx_holdoff_ctr != 0) begin
            rx_holdoff_ctr <= rx_holdoff_ctr - 1'b1;
        end
    end
    assign rx_valid_gated = rx_valid & (rx_holdoff_ctr == 0);

    // -----------------------------------------------------------------------
    // UART TX
    // -----------------------------------------------------------------------
    logic [7:0] tx_data;
    logic       tx_valid;
    logic       tx_ready;

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tx (
        .clk    (clk),
        .rst_n  (rst_n_sys),
        .tx_data (tx_data),
        .tx_valid(tx_valid),
        .tx_ready(tx_ready),
        .tx_pin  (uart_tx_pin)
    );

    // -----------------------------------------------------------------------
    // Config registers
    // -----------------------------------------------------------------------
    logic [7:0]             reg_wr_addr;
    logic [31:0]            reg_wr_data;
    logic                   reg_wr_en;
    logic [7:0]             reg_rd_addr;
    logic [31:0]            reg_rd_data;
    logic [1:0]             trig_mode;
    logic [PROBE_WIDTH-1:0] trig_mask;
    logic [PROBE_WIDTH-1:0] trig_value;
    logic [ADDR_BITS-1:0]   pre_trig_depth;
    logic [ADDR_BITS-1:0]   post_trig_depth;
    logic                   rle_en;
    logic [PROBE_WIDTH-1:0] probe_mask;

    config_regs #(
        .PROBE_WIDTH(PROBE_WIDTH),
        .ADDR_BITS  (ADDR_BITS)
    ) u_cfg (
        .clk           (clk),
        .rst_n         (rst_n_sys),
        .wr_addr       (reg_wr_addr),
        .wr_data       (reg_wr_data),
        .wr_en         (reg_wr_en),
        .rd_addr       (reg_rd_addr),
        .rd_data       (reg_rd_data),
        .trig_mode     (trig_mode),
        .trig_mask     (trig_mask),
        .trig_value    (trig_value),
        .pre_trig_depth (pre_trig_depth),
        .post_trig_depth(post_trig_depth),
        .rle_en        (rle_en),
        .probe_mask    (probe_mask)
    );

    // -----------------------------------------------------------------------
    // Trigger engine
    // -----------------------------------------------------------------------
    logic [PROBE_WIDTH-1:0] prev_data;
    logic                   trigger_fire;

    trigger_engine #(
        .PROBE_WIDTH(PROBE_WIDTH)
    ) u_trig (
        .probe_data  (probe_reg),
        .prev_data   (prev_data),
        .trig_mode   (trig_mode),
        .trig_mask   (trig_mask),
        .trig_value  (trig_value),
        .trigger_fire(trigger_fire)
    );

    // -----------------------------------------------------------------------
    // Circular capture buffer (BRAM)
    // -----------------------------------------------------------------------
    logic [ADDR_BITS-1:0]   wr_addr_buf, rd_addr_buf;
    logic [PROBE_WIDTH-1:0] wr_data_buf, rd_data_buf;
    logic                   wr_en_buf,   rd_en_buf;

    circular_capture_buffer #(
        .DEPTH(BUFFER_DEPTH),
        .WIDTH(PROBE_WIDTH)
    ) u_buf (
        .clk    (clk),
        .wr_addr(wr_addr_buf),
        .wr_data(wr_data_buf),
        .wr_en  (wr_en_buf),
        .rd_addr(rd_addr_buf),
        .rd_data(rd_data_buf),
        .rd_en  (rd_en_buf)
    );

    // -----------------------------------------------------------------------
    // Capture controller FSM
    // -----------------------------------------------------------------------
    logic                   do_arm, do_abort;
    logic                   o_armed, o_triggered, o_complete;
    logic [2:0]             fsm_state;
    logic                   rd_next;
    logic [PROBE_WIDTH-1:0] rd_sample_out;
    logic                   rd_sample_valid;
    logic                   rd_done;
    logic [ADDR_BITS-1:0]   trig_position;
    logic [ADDR_BITS:0]     total_samples;

    capture_controller_fsm #(
        .PROBE_WIDTH (PROBE_WIDTH),
        .BUFFER_DEPTH(BUFFER_DEPTH)
    ) u_fsm (
        .clk           (clk),
        .rst_n         (rst_n_sys),
        .probe_data    (probe_reg),
        .prev_data     (prev_data),
        .trigger_fire  (trigger_fire),
        .pre_trig_depth (pre_trig_depth),
        .post_trig_depth(post_trig_depth),
        .arm           (do_arm),
        .abort         (do_abort),
        .o_armed       (o_armed),
        .o_triggered   (o_triggered),
        .o_complete    (o_complete),
        .state_out     (fsm_state),
        .wr_addr       (wr_addr_buf),
        .wr_data       (wr_data_buf),
        .wr_en         (wr_en_buf),
        .rd_addr       (rd_addr_buf),
        .rd_en         (rd_en_buf),
        .rd_data       (rd_data_buf),
        .rd_next       (rd_next),
        .rd_sample_out (rd_sample_out),
        .rd_sample_valid(rd_sample_valid),
        .rd_done       (rd_done),
        .trig_position (trig_position),
        .total_samples (total_samples)
    );

    // -----------------------------------------------------------------------
    // RLE encoder
    // -----------------------------------------------------------------------
    logic [PROBE_WIDTH-1:0] rle_raw_data;
    logic                   rle_raw_valid;
    logic                   rle_raw_ready;
    logic                   rle_stream_end;
    logic [7:0]             rle_enc_byte;
    logic                   rle_enc_valid;
    logic                   rle_enc_ready;

    rle_encoder #(
        .PROBE_WIDTH(PROBE_WIDTH)
    ) u_rle (
        .clk       (clk),
        .rst_n     (rst_n_sys),
        .en        (rle_en),
        .raw_data  (rle_raw_data),
        .raw_valid (rle_raw_valid),
        .raw_ready (rle_raw_ready),
        .stream_end(rle_stream_end),
        .enc_byte  (rle_enc_byte),
        .enc_valid (rle_enc_valid),
        .enc_ready (rle_enc_ready)
    );

    // -----------------------------------------------------------------------
    // UART command parser
    // -----------------------------------------------------------------------
    uart_cmd_parser #(
        .PROBE_WIDTH (PROBE_WIDTH),
        .BUFFER_DEPTH(BUFFER_DEPTH)
    ) u_parser (
        .clk           (clk),
        .rst_n         (rst_n_sys),
        .rx_data       (rx_data),
        .rx_valid      (rx_valid_gated),
        .tx_data       (tx_data),
        .tx_valid      (tx_valid),
        .tx_ready      (tx_ready),
        .reg_wr_addr   (reg_wr_addr),
        .reg_wr_data   (reg_wr_data),
        .reg_wr_en     (reg_wr_en),
        .reg_rd_addr_i ('0),
        .reg_rd_addr   (reg_rd_addr),
        .reg_rd_data   (reg_rd_data),
        .do_arm        (do_arm),
        .do_abort      (do_abort),
        .fsm_state     (fsm_state),
        .trig_position (trig_position),
        .total_samples (total_samples),
        .rle_en        (rle_en),
        .rd_next       (rd_next),
        .rd_sample     (rd_sample_out),
        .rd_sample_valid(rd_sample_valid),
        .rd_done       (rd_done),
        .rle_raw_data  (rle_raw_data),
        .rle_raw_valid (rle_raw_valid),
        .rle_raw_ready (rle_raw_ready),
        .rle_stream_end(rle_stream_end),
        .rle_enc_byte  (rle_enc_byte),
        .rle_enc_valid (rle_enc_valid),
        .rle_enc_ready (rle_enc_ready)
    );

    // -----------------------------------------------------------------------
    // LED outputs (active high)
    // -----------------------------------------------------------------------
    assign led_armed     = o_armed;
    assign led_triggered = o_triggered;
    assign led_complete  = o_complete;

endmodule

`default_nettype wire
