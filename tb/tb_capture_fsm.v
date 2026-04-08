// tb_capture_fsm.v — cocotb wrapper that wires together:
//   capture_controller_fsm + circular_capture_buffer + trigger_engine
//
// All ports of the three modules are exposed so that
// test_capture_controller_fsm.py can drive and observe them directly.

`default_nettype none

module tb_capture_fsm #(
    parameter PROBE_WIDTH  = 8,
    parameter BUFFER_DEPTH = 64,
    parameter ADDR_BITS    = 6   // $clog2(64)
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // Probe data driven by cocotb
    input  wire [PROBE_WIDTH-1:0] probe_data,

    // Trigger configuration
    input  wire [1:0]             trig_mode,
    input  wire [PROBE_WIDTH-1:0] trig_mask,
    input  wire [PROBE_WIDTH-1:0] trig_value,

    // Capture configuration
    input  wire [ADDR_BITS-1:0]   pre_trig_depth,
    input  wire [ADDR_BITS-1:0]   post_trig_depth,

    // Control
    input  wire                   arm,
    input  wire                   abort,

    // Status / readout
    output wire                   o_armed,
    output wire                   o_triggered,
    output wire                   o_complete,
    output wire [2:0]             state_out,

    // Readout handshake
    input  wire                   rd_next,
    output wire [PROBE_WIDTH-1:0] rd_sample_out,
    output wire                   rd_sample_valid,
    output wire                   rd_done,

    // Capture metadata
    output wire [ADDR_BITS-1:0]   trig_position,
    output wire [ADDR_BITS:0]     total_samples
);

    // Internal wires between modules
    wire [PROBE_WIDTH-1:0] prev_data;
    wire                   trigger_fire;

    wire [ADDR_BITS-1:0]   wr_addr;
    wire [PROBE_WIDTH-1:0] wr_data;
    wire                   wr_en;

    wire [ADDR_BITS-1:0]   rd_addr;
    wire                   rd_en;
    wire [PROBE_WIDTH-1:0] rd_data;

    // -------------------------------------------------------------------
    // trigger_engine (combinational)
    // -------------------------------------------------------------------
    trigger_engine #(
        .PROBE_WIDTH(PROBE_WIDTH)
    ) u_trig (
        .probe_data  (probe_data),
        .prev_data   (prev_data),
        .trig_mode   (trig_mode),
        .trig_mask   (trig_mask),
        .trig_value  (trig_value),
        .trigger_fire(trigger_fire)
    );

    // -------------------------------------------------------------------
    // circular_capture_buffer
    // -------------------------------------------------------------------
    circular_capture_buffer #(
        .DEPTH    (BUFFER_DEPTH),
        .WIDTH    (PROBE_WIDTH),
        .ADDR_BITS(ADDR_BITS)
    ) u_buf (
        .clk    (clk),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .wr_en  (wr_en),
        .rd_addr(rd_addr),
        .rd_data(rd_data),
        .rd_en  (rd_en)
    );

    // -------------------------------------------------------------------
    // capture_controller_fsm
    // -------------------------------------------------------------------
    capture_controller_fsm #(
        .PROBE_WIDTH (PROBE_WIDTH),
        .BUFFER_DEPTH(BUFFER_DEPTH),
        .ADDR_BITS   (ADDR_BITS)
    ) u_fsm (
        .clk           (clk),
        .rst_n         (rst_n),
        .probe_data    (probe_data),
        .prev_data     (prev_data),
        .trigger_fire  (trigger_fire),
        .pre_trig_depth (pre_trig_depth),
        .post_trig_depth(post_trig_depth),
        .arm           (arm),
        .abort         (abort),
        .o_armed       (o_armed),
        .o_triggered   (o_triggered),
        .o_complete    (o_complete),
        .state_out     (state_out),
        .wr_addr       (wr_addr),
        .wr_data       (wr_data),
        .wr_en         (wr_en),
        .rd_addr       (rd_addr),
        .rd_en         (rd_en),
        .rd_data       (rd_data),
        .rd_next       (rd_next),
        .rd_sample_out (rd_sample_out),
        .rd_sample_valid(rd_sample_valid),
        .rd_done       (rd_done),
        .trig_position (trig_position),
        .total_samples (total_samples)
    );

endmodule

`default_nettype wire
